#! /usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray, InteractiveMarker, InteractiveMarkerControl
from interactive_markers import InteractiveMarkerServer
import json


class Control(Node):

    def __init__(self):
        super().__init__('control')

        self.marker_publisher = self.create_publisher(
            MarkerArray, "/detection_markers", 10
        )

        self.create_subscription(
            String, "/abs_detections", self.handle_detections, 10
        )

        self.overview_publisher = self.create_publisher(
            String, "/overview_messages", 10
        )

        self.timer_publisher = self.create_publisher(Marker, "/timer_marker", 10)

        self.detections = dict()
        self.start_time = None
        self.timer = None

        self.server = InteractiveMarkerServer(self, "start_button")
        self.make_button()

    def make_button(self):
        int_marker = InteractiveMarker()
        int_marker.header.frame_id = "map"
        int_marker.name = "start_button"
        int_marker.scale = 1.0
        int_marker.pose.position.z = 1.0
        int_marker.pose.orientation.w = 1.0

        box = Marker()
        box.type = Marker.CUBE
        box.scale.x = 1.0
        box.scale.y = 0.5
        box.scale.z = 0.2
        box.color.g = 1.0
        box.color.a = 1.0

        label = Marker()
        label.type = Marker.TEXT_VIEW_FACING
        label.scale.z = 0.3
        label.color.r = 1.0
        label.color.g = 1.0
        label.color.b = 1.0
        label.color.a = 1.0
        label.pose.position.z = 0.15
        label.text = "START"

        control = InteractiveMarkerControl()
        control.interaction_mode = InteractiveMarkerControl.BUTTON
        control.always_visible = True
        control.markers.append(box)
        control.markers.append(label)
        int_marker.controls.append(control)

        self.server.insert(int_marker, feedback_callback=self.handle_button)
        self.server.applyChanges()

    def handle_button(self, feedback):
        if feedback.event_type != feedback.BUTTON_CLICK:
            return

        msg = String()
        msg.data = "start"
        self.overview_publisher.publish(msg)

        if self.timer is None:
            self.start_time = self.get_clock().now()
            self.timer = self.create_timer(0.1, self.publish_timer)

    def publish_timer(self):
        elapsed = (self.get_clock().now() - self.start_time).nanoseconds / 1e9

        marker = Marker()
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.header.frame_id = "map"
        marker.ns = "timer"
        marker.id = 0
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD
        marker.pose.position.z = 2.0
        marker.pose.orientation.w = 1.0
        marker.scale.z = 0.4
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 1.0
        marker.color.a = 1.0
        marker.text = f"{elapsed:.1f}"
        self.timer_publisher.publish(marker)

    def handle_detections(self, msg):
        json_msg = json.loads(msg.data)
        for marker_id, coords in json_msg.items():
            self.detections[int(marker_id)] = [float(c) for c in coords]

        self.publish_markers()

    def publish_markers(self):
        array = MarkerArray()
        stamp = self.get_clock().now().to_msg()

        for marker_id, (x, y, z) in self.detections.items():
            sphere = Marker()
            sphere.header.stamp = stamp
            sphere.header.frame_id = "map"
            sphere.ns = "detections"
            sphere.id = marker_id * 2
            sphere.type = Marker.SPHERE
            sphere.action = Marker.ADD
            sphere.pose.position.x = x
            sphere.pose.position.y = y
            sphere.pose.position.z = z
            sphere.pose.orientation.w = 1.0
            sphere.scale.x = 0.2
            sphere.scale.y = 0.2
            sphere.scale.z = 0.2
            sphere.color.r = 1.0
            sphere.color.a = 1.0
            array.markers.append(sphere)

            text = Marker()
            text.header = sphere.header
            text.ns = "detection_labels"
            text.id = marker_id * 2 + 1
            text.type = Marker.TEXT_VIEW_FACING
            text.action = Marker.ADD
            text.pose.position.x = x
            text.pose.position.y = y
            text.pose.position.z = z + 0.25
            text.pose.orientation.w = 1.0
            text.scale.z = 0.2
            text.color.b = 1.0
            text.color.a = 1.0
            text.text = f"ID: {marker_id}"
            array.markers.append(text)

        self.marker_publisher.publish(array)


def main(args=None):
    rclpy.init(args=args)

    control_node = Control()

    rclpy.spin(control_node)

    control_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
