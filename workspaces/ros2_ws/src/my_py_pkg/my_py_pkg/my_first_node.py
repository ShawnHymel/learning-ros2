#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

class MyNode(Node):

    def __init__(self):
        super().__init__("my_node")
        self.counter_ = 0
        self.get_logger().info("Hello, world!")
        self.create_timer(1.0, self.timer_callback)

    def timer_callback(self):
        self.get_logger().info("Timer callback executed: " + str(self.counter_))
        self.counter_ += 1

def main(args=None):

    # Initialize the ROS2 Python client library
    rclpy.init(args=args)

    # Create a node
    node = MyNode()

    # Do nothing
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()