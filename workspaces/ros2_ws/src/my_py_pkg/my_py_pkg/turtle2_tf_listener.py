#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import tf2_ros
import math

class TurtleFollower(Node):
    def __init__(self):
        super().__init__('turtle2_tf_listener')
        self.cmd_pub = self.create_publisher(Twist, '/turtle2/cmd_vel', 10)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.timer = self.create_timer(0.1, self.follow)

    def follow(self):
        try:
            trans = self.tf_buffer.lookup_transform('turtle2', 'turtle1', rclpy.time.Time())
            dx = trans.transform.translation.x
            dy = trans.transform.translation.y
            angle_to_target = math.atan2(dy, dx)

            twist = Twist()
            twist.linear.x = 2.0 * math.sqrt(dx**2 + dy**2)
            twist.angular.z = 4.0 * angle_to_target
            self.cmd_pub.publish(twist)

        except Exception as e:
            self.get_logger().warn(f'Could not transform: {e}')

def main():
    rclpy.init()
    node = TurtleFollower()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
