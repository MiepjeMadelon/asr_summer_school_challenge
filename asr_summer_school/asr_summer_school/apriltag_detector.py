#! /usr/bin/env python3

import rclpy
import math
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.time import Time
import tf2_ros

from geometry_msgs.msg import Pose, PoseArray
from std_msgs.msg import Int32MultiArray
from landmark_msgs.msg import LandmarkArray
from tf2_geometry_msgs import do_transform_point
from geometry_msgs.msg import Point, PointStamped
#from asr_summer_school.msg import AbsoluteDetections
#from asr_summer_school.msg import AbsoluteDetection

class ApriltagSubscriber(Node):

    def __init__(self):
        super().__init__('apriltag_subscriber')

        self.markers = {}

        tf_cache_duration = 10.0  # seconds
        self.tf_buffer = tf2_ros.Buffer(cache_time=Duration(seconds=tf_cache_duration))
        # spin_thread=True: the buffer needs its own thread, otherwise a
        # blocking lookup timeout inside a callback can never resolve.
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self, spin_thread=True)
        
        self.subscription = self.create_subscription(
            LandmarkArray,
            '/camera/landmarks',
            self.listener_callback,
            10)

        self.pose_pub = self.create_publisher(PoseArray, '/tag_poses_map', 10)
        self.id_pub = self.create_publisher(Int32MultiArray, '/tag_ids', 10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        for tag in msg.landmarks:
            self.marker_detected(tag, msg.header)
            
    def marker_detected(self, tag, header):
         if tag.id not in self.markers:
             self.get_logger().info('New marker #%d' % tag.id)
             
             # /home/mauro/ros_ws/src/asr_summer_school_challenge/turtlebot3_perception/turtlebot3_perception/turtlebot3_perception/detection2landmark.py
             # https://fer.gs/ros2_cookbook/client_libraries/rclpy/tf2.html#transformations
             # detection2landmark expresses range/bearing in its
             # robot_base_frame and declares it in the array header.
             source_frame = header.frame_id
             target_frame = 'map'
             try:
                 transformation = self.tf_buffer.lookup_transform(
                     target_frame,
                     source_frame,
                     Time.from_msg(header.stamp),
                     timeout=Duration(seconds=0.2)
                 )
             except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                 self.get_logger().error(f"Unable to find the transformation from {source_frame} to {target_frame}")
                 return
                 
             point_source = PointStamped()
             point_source.header = header
             point_source.point = Point(x=float(tag.range) * math.cos(tag.bearing),
                                        y=float(tag.range) * math.sin(tag.bearing))
             point_target = do_transform_point(point_source, transformation)

             temp = {"x": point_target.point.x, "y": point_target.point.y}
             self.markers[tag.id] = temp
             self.publish()
             
    def publish(self):
        ids = sorted(self.markers.keys())

        im = Int32MultiArray()
        im.data = ids
        self.id_pub.publish(im)

        pa = PoseArray()
        pa.header.frame_id = 'map'
        pa.header.stamp = self.get_clock().now().to_msg()
        for tid in ids:
            p = Pose()
            p.position.x = self.markers[tid]['x']
            p.position.y = self.markers[tid]['y']
            p.orientation.w = 1.0
            pa.poses.append(p)
        self.pose_pub.publish(pa)


def main(args=None):
    rclpy.init(args=args)

    apriltag_subscriber = ApriltagSubscriber()

    rclpy.spin(apriltag_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    apriltag_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
