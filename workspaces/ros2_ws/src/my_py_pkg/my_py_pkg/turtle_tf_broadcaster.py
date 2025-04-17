#!/usr/bin/env python3

import math

from geometry_msgs.msg import TransformStamped
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from turtlesim.msg import Pose
import tf2_ros

from my_py_pkg import util

class TurtleTFBroadcaster(Node):

    def __init__(self, child_frame, parent_frame):
        """Constructor"""
        super().__init__("turtle_tf_broadcaster")

        # Save arguments
        self._child = child_frame
        self._parent = parent_frame

        # Subscribe to child turtle position
        self._sub = self.create_subscription(
            Pose,
            "/" + child_frame + "/pose",
            self._broadcast,
            10
        )

        # Create a transform broadcaster
        self._broadcaster = tf2_ros.TransformBroadcaster(self)

        # Say we've started
        self.get_logger().info("Turtle transform broadcaster started")

    def _broadcast(self, msg):
        """Broadcast turtle pose as a transform"""

        # Construct broadcast message header
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self._parent
        t.child_frame_id = self._child

        # Add translation (position) info
        t.transform.translation.x = msg.x
        t.transform.translation.y = msg.y
        t.transform.translation.z = 0.0

        # Add rotation info
        q = util.quaternion_from_euler(0, 0, msg.theta)
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]

        # Send out transform
        self._broadcaster.sendTransform(t)

def main():
    try:
        rclpy.init()
        node = TurtleTFBroadcaster("turtle1", "world")
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass

if __name__ == '__main__':
    main()