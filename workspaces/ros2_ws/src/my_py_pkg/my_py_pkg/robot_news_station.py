#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from example_interfaces.msg import String

class RobotNewsStationNode(Node):
    def __init__(self):
        super().__init__("robot_news_station")

        # Set up parameters
        self.declare_parameter("robot_name", rclpy.Parameter.Type.STRING)
        self.declare_parameter("timer_period", 1.0)

        # Set to attributes
        self.robot_name_ = self.get_parameter("robot_name").value
        self.timer_period_ = self.get_parameter("timer_period").value

        # Create publisher
        self.publisher_ = self.create_publisher(String, "robot_news", 10)
        
        # Create periodic timer
        self.timer_ = self.create_timer(self.timer_period_, self.publish_news)
        
        # Show the node setup is done
        self.get_logger().info("Robot News Station has been started.")

    def publish_news(self):
        msg = String()
        msg.data = "Hi, this is " + self.robot_name_ + " from the robot news station"
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = RobotNewsStationNode()
    rclpy.spin(node)
    rclpy.shutdown()
