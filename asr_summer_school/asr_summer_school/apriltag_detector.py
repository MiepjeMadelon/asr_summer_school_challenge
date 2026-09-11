#! /usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
import tf2_ros

from geometry_msgs.msg import Pose, PoseArray
from std_msgs.msg import Int32MultiArray
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

        self.pose_pub = self.create_publisher(PoseArray, '/tag_poses_map', 10)
        self.id_pub = self.create_publisher(Int32MultiArray, '/tag_ids', 10)
        self.create_subscription(String, '/overview_messages', self.cmd_cb, 10)
        self.started = False
        self.subscription  # prevent unused variable warning

    def cmd_cb(self, msg):
        if msg.data == 'start':
            self.started = True
        elif msg.data == 'stop':
            self.started = False

    def listener_callback(self, msg):
        if not self.started:
            return
        for tag in msg.landmarks:
            self.marker_detected(tag)
            
    def marker_detected(self, tag):
         if tag.id not in self.markers:
             self.get_logger().info('New marker #%d' % tag.id)
             
             # /home/mauro/ros_ws/src/asr_summer_school_challenge/turtlebot3_perception/turtlebot3_perception/turtlebot3_perception/detection2landmark.py
             # https://fer.gs/ros2_cookbook/client_libraries/rclpy/tf2.html#transformations
             source_frame = f"tag36h11:{tag.id}"
             target_frame = 'map'
             try:
                 transformation = self.tf_buffer.lookup_transform(
                     target_frame,
                     source_frame,
                     rclpy.time.Time()
                 )
             except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                 self.get_logger().error(f"Unable to find the transformation from {source_frame} to {target_frame}")
                 return
                 
             #point_source = PointStamped()
             #point_source.header.frame_id = source_frame
             #point_source.point = Node.get_position_in_parent("map", "odom")
             #point_target = do_transform_point(point_source, transformation)
             point_target = transformation.transform.translation

             temp = {"x": point_target.x, "y": point_target.y, "z": point_target.z}
             self.markers[tag.id] = temp
             self.publish()
             
    def publish(self):
        ids = sorted(self.markers.keys())

        pa = PoseArray()
        pa.header.frame_id = 'map'
        pa.header.stamp = self.get_clock().now().to_msg()
        for tid in ids:
            p = Pose()
            p.position.x = self.markers[tid]['x']
            p.position.y = self.markers[tid]['y']
            p.position.z = self.markers[tid]['z']
            p.orientation.w = 1.0
            pa.poses.append(p)
        self.pose_pub.publish(pa)

        im = Int32MultiArray()
        im.data = ids
        self.id_pub.publish(im)


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
