#! /usr/bin/env python3

import rclpy
from rclpy.node import Node

from apriltag_msgs.msg import AprilTagDetectionArray
from asr_summer_school import AbsoluteDetections

class ApriltagSubscriber(Node):

    markers = {}

    def __init__(self):
        super().__init__('apriltag_subscriber')
        self.subscription = self.create_subscription(
            AprilTagDetectionArray,
            '/camera/detections',
            self.listener_callback,
            10)
        self.publisher_ = self.create_publisher(String, '/abs_detections', 10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        for tag in msg.detections:
            self.marker_detected(tag)
            
    def marker_detected(self, tag):
         if tag.id not in self.markers:
             self.get_logger().info('New marker #%d' % tag.id)
             self.markers[tag.id] = tag.homography


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
