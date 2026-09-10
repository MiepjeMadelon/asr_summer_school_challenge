#! /usr/bin/env python3

import rclpy
import json
from rclpy.node import Node
from rclpy.duration import Duration
import tf2_ros

from std_msgs.msg import String
from apriltag_msgs.msg import AprilTagDetectionArray
from landmark_msgs.msg import LandmarkArray
from tf2_geometry_msgs import do_transform_point
from geometry_msgs.msg import Point, PointStamped
#from asr_summer_school.msg import AbsoluteDetections
#from asr_summer_school.msg import AbsoluteDetection

class ApriltagSubscriber(Node):

    markers = {}

    def __init__(self):
        super().__init__('apriltag_subscriber')
        
        tf_cache_duration = 10.0  # seconds
        self.tf_buffer = tf2_ros.Buffer(cache_time=Duration(seconds=tf_cache_duration))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        self.subscription = self.create_subscription(
            LandmarkArray,
            '/camera/landmarks',
            self.listener_callback,
            10)
        self.publisher_ = self.create_publisher(String, '/abs_detections', 10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        for tag in msg.landmarks:
            self.marker_detected(tag)
            
    def marker_detected(self, tag):
         if tag.id not in self.markers:
             self.get_logger().info('New marker #%d' % tag.id)
             
             # /home/mauro/ros_ws/src/asr_summer_school_challenge/turtlebot3_perception/turtlebot3_perception/turtlebot3_perception/detection2landmark.py
             # https://fer.gs/ros2_cookbook/client_libraries/rclpy/tf2.html#transformations
             source_frame = 'camera_color_optical_frame'
             target_frame = 'odom'
             try:
                 transformation = self.tf_buffer.lookup_transform(
                     target_frame,
                     source_frame,
                     rclpy.time.Time()
                 )
             except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                 self.get_logger().error(f"Unable to find the transformation from {source_frame} to {target_frame}")
                 pass
                 
             point_source = PointStamped()
             point_source.point = Point(x=0.1, y=1.2, z=2.3)
             point_target = do_transform_point(point_source, transformation)
             
             temp = {"x": point_target.point.x, "y": point_target.point.y, "z": point_target.point.z}
             self.markers[tag.id] = temp
             self.publish()
             
    def publish(self):
        msg = String()
        msg.data = json.dumps(self.markers)
        self.publisher_.publish(msg)


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
