#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from example_interfaces.msg import Int64

class NumPubNode(Node):
    def __init__(self):
        super().__init__("number_publisher")

        # Set up parameters
        self.declare_parameter("number", 2)
        self.declare_parameter("timer_period", 1.0)

        # Set to attributes
        self.number_ = self.get_parameter("number").value
        self.timer_period_ = self.get_parameter("timer_period").value

        # Configure callback for runtime parameter update
        self.add_post_set_parameters_callback(self.parameters_callback)

        self.publisher_ = self.create_publisher(Int64, "number", 10)
        self.timer_ = self.create_timer(self.timer_period_, self.publish_number)
        self.get_logger().info("Num Pub has been started.")

    def publish_number(self):
        msg = Int64()
        msg.data = self.number_
        self.publisher_.publish(msg)

    def parameters_callback(self, params: list[Parameter]):
        for param in params:
            if param.name == "number":
                self.number_ = param.value
            if param.name == "timer_period":
                self.timer_period_ = param.value
                self.timer_.cancel()
                self.timer_ = self.create_timer(self.timer_period_, self.publish_number)

def main(args=None):
    rclpy.init(args=args)
    node = NumPubNode()
    rclpy.spin(node)
    rclpy.shutdown()
