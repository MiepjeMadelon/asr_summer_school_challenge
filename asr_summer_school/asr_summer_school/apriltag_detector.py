#! /usr/bin/env python3
import math

import rclpy
import tf2_ros
from geometry_msgs.msg import Point, PointStamped, Pose, PoseArray
from landmark_msgs.msg import LandmarkArray
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from std_msgs.msg import Int32MultiArray
from tf2_geometry_msgs import do_transform_point

MAX_SAMPLES = 20   # osservation per tag


class ApriltagSubscriber(Node):

    def __init__(self):
        super().__init__('apriltag_subscriber')

        self.declare_parameter('max_range', 2.0)
        self.max_range = self.get_parameter('max_range').value

        # id of tag -> list observation (x, y) just in map frame
        self.samples = {}

        self.tf_buffer = tf2_ros.Buffer(cache_time=Duration(seconds=10.0))
        # spin_thread=True: il buffer ha bisogno di un thread suo, altrimenti
        # una lookup con timeout dentro una callback non si risolve mai.
        self.tf_listener = tf2_ros.TransformListener(
            self.tf_buffer, self, spin_thread=True)

        self.create_subscription(
            LandmarkArray, '/camera/landmarks', self.listener_callback, 10)

        self.pose_pub = self.create_publisher(PoseArray, '/tag_poses_map', 10)
        self.id_pub = self.create_publisher(Int32MultiArray, '/tag_ids', 10)

    def listener_callback(self, msg):
        updated = False
        for tag in msg.landmarks:
            if self.marker_detected(tag, msg.header):
                updated = True
        if updated:
            self.publish()

    def marker_detected(self, tag, header):
        
        if tag.range > self.max_range:
            return False

        try:
            # lookup all'istante del messaggio, non all'ultima TF disponibile
            transformation = self.tf_buffer.lookup_transform(
                'map', header.frame_id, Time.from_msg(header.stamp),
                timeout=Duration(seconds=0.2))
        except tf2_ros.TransformException as exc:
            self.get_logger().warn(
                f'TF {header.frame_id} -> map non disponibile: {exc}')
            return False

        # detection2landmark da' range/bearing nel frame dichiarato nell'header
        point_source = PointStamped()
        point_source.header = header
        point_source.point = Point(x=tag.range * math.cos(tag.bearing),
                                   y=tag.range * math.sin(tag.bearing),
                                   z=0.0)
        point_target = do_transform_point(point_source, transformation)

        samples = self.samples.setdefault(tag.id, [])
        if not samples:
            self.get_logger().info(f'Nuovo tag #{tag.id}')
        samples.append((point_target.point.x, point_target.point.y))
        del samples[:-MAX_SAMPLES]
        return True

    def publish(self):
        ids = sorted(self.samples.keys())

        # gli id vanno pubblicati prima delle pose: chi ascolta le accoppia
        # per indice e deve gia' avere la lista aggiornata quando arrivano.
        im = Int32MultiArray()
        im.data = [int(i) for i in ids]
        self.id_pub.publish(im)

        pa = PoseArray()
        pa.header.frame_id = 'map'
        pa.header.stamp = self.get_clock().now().to_msg()
        for tid in ids:
            samples = self.samples[tid]
            p = Pose()
            # media delle osservazioni: la prima e' tipicamente la peggiore
            p.position.x = sum(x for x, _ in samples) / len(samples)
            p.position.y = sum(y for _, y in samples) / len(samples)
            p.orientation.w = 1.0
            pa.poses.append(p)
        self.pose_pub.publish(pa)


def main(args=None):
    rclpy.init(args=args)
    apriltag_subscriber = ApriltagSubscriber()
    try:
        rclpy.spin(apriltag_subscriber)
    except KeyboardInterrupt:
        pass
    finally:
        apriltag_subscriber.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
