#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose
from geometry_msgs.msg import TransformStamped, Quaternion
import tf2_ros

class TurtleTfBroadcaster(Node):
    def __init__(self):
        super().__init__('turtle1_tf_broadcaster')
        self.subscription = self.create_subscription(
            Pose,
            '/turtle1/pose',
            self.handle_turtle_pose,
            10)
        self.broadcaster = tf2_ros.TransformBroadcaster(self)
        self.get_logger().info("Turtle 1 broadcaster node started")

    def quaternion_from_euler(self, roll, pitch, yaw):
        """
        Convert Euler angles (in radians) to quaternion (x, y, z, w)
        
        https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
        """

        # Calculate half-angles
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)

        # Calculate quaternion components
        qw = cr * cp * cy + sr * sp * sy
        qx = sr * cp * cy - cr * sp * sy
        qy = cr * sp * cy + sr * cp * sy
        qz = cr * cp * sy - sr * sp * cy

        return (qx, qy, qz, qw)

    def handle_turtle_pose(self, msg):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'world'
        t.child_frame_id = 'turtle1'
        t.transform.translation.x = msg.x
        t.transform.translation.y = msg.y
        t.transform.translation.z = 0.0

        quat = self.quaternion_from_euler(0, 0, msg.theta)
        t.transform.rotation.x = quat[0]
        t.transform.rotation.y = quat[1]
        t.transform.rotation.z = quat[2]
        t.transform.rotation.w = quat[3]

        self.broadcaster.sendTransform(t)

def main():
    rclpy.init()
    node = TurtleTfBroadcaster()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
