#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from example_interfaces.msg import Int64

class RobotNewsStationNode(Node):
    def __init__(self):
        super().__init__("number_publisher")
        self.robot_name_ = "C3PO"
        self.publisher_ = self.create_publisher(Int64, "number", 10)
        self.timer_ = self.create_timer(1.0, self.publish_number)
        self.get_logger().info("Num Pub has been started.")

    def publish_number(self):
        msg = Int64()
        msg.data = 2
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = RobotNewsStationNode()
    rclpy.spin(node)
    rclpy.shutdown()
