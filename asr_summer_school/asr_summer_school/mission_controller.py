#! /usr/bin/env python3
import json
import math
import os
import subprocess
import time
from collections import deque
from enum import Enum

import numpy as np
import rclpy
from geometry_msgs.msg import PoseArray, PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from nav_msgs.msg import OccupancyGrid
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile,
                       QoSReliabilityPolicy)
from std_msgs.msg import Int32MultiArray, String
from tf2_ros import Buffer, TransformListener


class State(Enum):
    EXPLORING = 1
    RETURNING = 2
    DONE = 3


class MissionController(Node):

    def __init__(self):
        super().__init__('mission_controller')

        # ---------------- Parametrs -----------
        d = self.declare_parameter
        d('mission_duration', 240.0)   
        d('n_tags_target', 12)          
        d('avg_speed', 0.12)            
        d('return_margin', 15.0)        
        d('safety_factor', 1.5)         
        d('goal_timeout', 30.0)         
        d('min_cluster', 6)             
        d('min_obstacle_dist', 0.13)    
        d('unknown_buffer', 0.15)
        d('output_dir', os.path.expanduser('~/challenge_output'))

        g = lambda n: self.get_parameter(n).value
        self.duration = g('mission_duration')
        self.n_target = g('n_tags_target')
        self.speed = max(float(g('avg_speed')), 1e-3)
        self.margin = g('return_margin')
        self.safety = g('safety_factor')
        self.goal_timeout = g('goal_timeout')
        self.min_cluster = g('min_cluster')
        self.min_obs = g('min_obstacle_dist')
        self.unk_buf = g('unknown_buffer')
        self.outdir = g('output_dir')

        # ---------------- state ----------------
        self.state = State.EXPLORING
        self.map_msg = None
        self.home = None
        self.t0 = None
        self.goal = None
        self.goal_t0 = None
        self.failed = []           
        self.tags = {}              
        self.saved = False

  
        qos = QoSProfile(depth=1,
                         reliability=QoSReliabilityPolicy.RELIABLE,
                         durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                         history=QoSHistoryPolicy.KEEP_LAST)
        self.create_subscription(OccupancyGrid, '/map', self.map_cb, qos)
        self.create_subscription(PoseArray, '/tag_poses_map', self.tags_cb, 10)
        self.create_subscription(Int32MultiArray, '/tag_ids', self.ids_cb, 10)
        self.create_subscription(String, '/overview_messages', self.cmd_cb, 10)
        self.cmd_pub = self.create_publisher(String, '/overview_messages', 10)

        self.tf = Buffer()
        self.tf_listener = TransformListener(self.tf, self, spin_thread=True)
        self.nav = BasicNavigator()
        # BasicNavigator crea un nodo suo: senza questo leggerebbe il
        # wall clock mentre il resto del sistema usa il clock di Gazebo.
        self.nav.set_parameters([Parameter(
            'use_sim_time', Parameter.Type.BOOL,
            self.get_parameter('use_sim_time').value)])

        self.last_ids = []
        self.last_poses = []
        self.cmd = None


    def cmd_cb(self, msg):
        self.cmd = msg.data

    def map_cb(self, msg):
        self.map_msg = msg

    def ids_cb(self, msg):
        self.last_ids = list(msg.data)
        self.sync_tags()

    def tags_cb(self, msg):
        self.last_poses = msg.poses
        self.sync_tags()

    def sync_tags(self):
        if not self.last_ids or len(self.last_ids) != len(self.last_poses):
            return
        for tid, p in zip(self.last_ids, self.last_poses):
            if tid not in self.tags:
                self.get_logger().info(
                    f'>>> TAG {tid} @ ({p.position.x:.2f}, {p.position.y:.2f})  '
                    f'[{len(self.tags) + 1}/{self.n_target}]')
            self.tags[tid] = (p.position.x, p.position.y)

    def pose(self):
        for f in ('base_footprint', 'base_link'):
            try:
                t = self.tf.lookup_transform('map', f, rclpy.time.Time(),
                                             timeout=Duration(seconds=0.3))
            except Exception:
                continue
            p = PoseStamped()
            p.header.frame_id = 'map'
            p.header.stamp = self.get_clock().now().to_msg()
            p.pose.position.x = t.transform.translation.x
            p.pose.position.y = t.transform.translation.y
            p.pose.orientation.w = 1.0
            return p
        return None

    def elapsed(self):
        return (self.get_clock().now() - self.t0).nanoseconds / 1e9

    def must_return(self, robot):
        d = math.hypot(robot.pose.position.x - self.home.pose.position.x,
                       robot.pose.position.y - self.home.pose.position.y)
        # needed = (d * 1.4 / self.speed) * self.safety + self.margin
        needed = 60.0
        left = self.duration - self.elapsed()
        if left <= needed:
            self.get_logger().warn(
                f'RIENTRO: restano {left:.0f}s, servono {needed:.0f}s '
                f'(distanza {d:.1f}m)')
            return True
        return False


    def next_goal(self, robot):

        if self.map_msg is None:
            return None

        info = self.map_msg.info
        grid = np.array(self.map_msg.data, dtype=np.int16).reshape(
            info.height, info.width)

        free = (grid >= 0) & (grid <= 20)
        unknown = (grid == -1)

        k = max(1, int(round(self.unk_buf / info.resolution)))
        cnt = np.zeros((info.height, info.width), dtype=np.int16)
        for dr in range(-k, k + 1):
            for dc in range(-k, k + 1):
                cnt[max(0, dr):info.height + min(0, dr),
                    max(0, dc):info.width + min(0, dc)] += \
                    unknown[max(0, -dr):info.height + min(0, -dr),
                            max(0, -dc):info.width + min(0, -dc)]
        nb = cnt >= (2 * k + 1) ** 2 // 4


        occ = (grid >= 65)
        wall = np.zeros_like(occ)
        pad = max(1, int(round(self.min_obs / info.resolution)))
        for dr in range(-pad, pad + 1):
            for dc in range(-pad, pad + 1):
                wall[max(0, dr):info.height + min(0, dr),
                     max(0, dc):info.width + min(0, dc)] |= \
                    occ[max(0, -dr):info.height + min(0, -dr),
                        max(0, -dc):info.width + min(0, -dc)]

        rx, ry = robot.pose.position.x, robot.pose.position.y
        rr = int((ry - info.origin.position.y) / info.resolution)
        rc = int((rx - info.origin.position.x) / info.resolution)
        if not (0 <= rr < info.height and 0 <= rc < info.width):
            return None

        trav = free & ~wall
        if not trav[rr, rc]:
            trav = free
        reach = np.zeros_like(trav)
        reach[rr, rc] = True
        while True:
            nxt = reach.copy()
            nxt[1:, :] |= reach[:-1, :]
            nxt[:-1, :] |= reach[1:, :]
            nxt[:, 1:] |= reach[:, :-1]
            nxt[:, :-1] |= reach[:, 1:]
            nxt &= trav
            if np.array_equal(nxt, reach):
                break
            reach = nxt

        mask = free & nb & ~wall & reach

        seen = np.zeros_like(mask, dtype=bool)
        best, best_score = None, -1e9

        for r0, c0 in np.argwhere(mask):
            if seen[r0, c0]:
                continue
            cells, q = [], deque([(r0, c0)])
            seen[r0, c0] = True
            while q:
                r, c = q.popleft()
                cells.append((r, c))
                for dr in (-1, 0, 1):
                    for dc in (-1, 0, 1):
                        nr, nc = r + dr, c + dc
                        if (0 <= nr < info.height and 0 <= nc < info.width
                                and mask[nr, nc] and not seen[nr, nc]):
                            seen[nr, nc] = True
                            q.append((nr, nc))

            if len(cells) < self.min_cluster:
                continue

            mr = sum(c[0] for c in cells) / len(cells)
            mc = sum(c[1] for c in cells) / len(cells)
            cand = [p for p in cells if math.hypot(
                info.origin.position.x + (p[1] + 0.5) * info.resolution - rx,
                info.origin.position.y + (p[0] + 0.5) * info.resolution - ry) >= 0.3]
            if not cand:
                continue
            cr, cc = min(cand, key=lambda p: (p[0] - mr) ** 2 + (p[1] - mc) ** 2)
            wx = info.origin.position.x + (cc + 0.5) * info.resolution
            wy = info.origin.position.y + (cr + 0.5) * info.resolution

            if any(math.hypot(wx - fx, wy - fy) < 0.4 for fx, fy in self.failed):
                continue
            dist = math.hypot(wx - rx, wy - ry)
            if dist < 0.3:
                continue
            score = math.sqrt(len(cells)) - 2.0 * dist
            if score > best_score:
                best_score, best = score, (wx, wy)

        if best is None:
            return None

        g = PoseStamped()
        g.header.frame_id = 'map'
        g.header.stamp = self.get_clock().now().to_msg()
        g.pose.position.x = best[0]
        g.pose.position.y = best[1]
        g.pose.orientation.w = 1.0
        return g


    def save(self):
        if self.saved:
            return
        self.saved = True
        self.cmd_pub.publish(String(data='stop'))
        os.makedirs(self.outdir, exist_ok=True)

        path = os.path.join(self.outdir, 'semantic_map.json')
        with open(path, 'w') as f:
            json.dump({'n_tags': len(self.tags),
                       'tags': [{'id': int(k), 'x': round(v[0], 3),
                                 'y': round(v[1], 3)}
                                for k, v in sorted(self.tags.items())]},
                      f, indent=2)

        subprocess.run(['ros2', 'run', 'nav2_map_server', 'map_saver_cli',
                        '-f', os.path.join(self.outdir, 'map')],
                       timeout=40, check=False)

        self.get_logger().info(f'Salvati {len(self.tags)} tag in {self.outdir}')


    def reset(self):
        self.state = State.EXPLORING
        self.saved = False
        self.goal = None
        self.cmd = None

    def run(self):
        self.get_logger().info('Attendo mappa e TF...')
        while rclpy.ok() and (self.map_msg is None or self.pose() is None):
            time.sleep(0.5)

        # il cronometro parte solo quando Nav2 e' pronto ad accettare goal
        self.get_logger().info('Attendo Nav2...')
        self.nav.nav_to_pose_client.wait_for_server()

        self.get_logger().info('Attendo START...')
        while rclpy.ok() and self.cmd != 'start':
            time.sleep(0.1)

        self.home = self.pose()
        self.t0 = self.get_clock().now()
        self.get_logger().info(
            f'HOME=({self.home.pose.position.x:.2f}, '
            f'{self.home.pose.position.y:.2f})  budget={self.duration:.0f}s')

        while rclpy.ok() and self.state != State.DONE:
            time.sleep(0.1)
            robot = self.pose()
            if robot is None:
                continue

            if self.cmd == 'stop':
                self.get_logger().info('STOP')
                self.nav.cancelTask()
                self.save()
                self.state = State.DONE
                continue

            if self.cmd == 'home' and self.state == State.EXPLORING:
                self.get_logger().info('RIENTRO (comando HOME)')
                self.nav.cancelTask()
                self.goal = None
                self.state = State.RETURNING
                self.nav.goToPose(self.home)
                continue

            if self.state == State.EXPLORING:

    
                stop = None
                if self.must_return(robot):
                    stop = 'tempo esaurito'

                if stop:
                    self.get_logger().info(f'RIENTRO ({stop})')
                    self.nav.cancelTask()
                    self.goal = None
                    self.state = State.RETURNING
                    self.nav.goToPose(self.home)
                    continue

                if self.goal is None:
                    g = self.next_goal(robot)
                    if g is None:
                        self.get_logger().info('RIENTRO (mappa completa)')
                        self.state = State.RETURNING
                        self.nav.goToPose(self.home)
                        continue
                    self.goal = g
                    self.goal_t0 = self.get_clock().now()
                    self.nav.goToPose(g)
                    self.get_logger().info(
                        f'GOAL ({g.pose.position.x:.2f}, {g.pose.position.y:.2f})  '
                        f't={self.elapsed():.0f}/{self.duration:.0f}s  '
                        f'tag={len(self.tags)}')

                elif self.nav.isTaskComplete():
                    if self.nav.getResult() != TaskResult.SUCCEEDED:
                        self.failed.append((self.goal.pose.position.x,
                                            self.goal.pose.position.y))
                    self.goal = None

                elif (self.get_clock().now() - self.goal_t0).nanoseconds / 1e9 > self.goal_timeout:

                    self.get_logger().warn('Goal troppo lento, ne scelgo un altro')
                    self.nav.cancelTask()
                    self.failed.append((self.goal.pose.position.x,
                                        self.goal.pose.position.y))
                    self.goal = None

            elif self.state == State.RETURNING:
                if self.nav.isTaskComplete():
                    d = math.hypot(robot.pose.position.x - self.home.pose.position.x,
                                   robot.pose.position.y - self.home.pose.position.y)
                    self.get_logger().info(
                        f'A {d:.2f}m da HOME  t={self.elapsed():.0f}s')
                    if d > 0.4 and self.elapsed() < self.duration - 10:
                        self.nav.goToPose(self.home)   # riprova
                    else:
                        self.save()
                        self.state = State.DONE


def main(args=None):
    rclpy.init(args=args)
    node = MissionController()
    try:
        while rclpy.ok():
            node.run()
            node.reset()
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.save()
        except Exception:
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()