#!/usr/bin/env python3

from geometry_msgs.msg import PointStamped
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
import tf2_ros
import tf2_geometry_msgs

class PointProjector(Node):

    def __init__(self, frame):
        """Constructor"""
        super().__init__("turtle_tf_listener")

        # Initialize fields
        self._frame = frame
        self._buffer = tf2_ros.Buffer()
        self._listener = tf2_ros.TransformListener(self._buffer, self)
        self._timer = self.create_timer(0.5, self._project_point)

        # Say we've started
        self.get_logger().info("Point projector started")

    def _project_point(self):
        """Print coordinates of point 1 unit in front of frame"""

        # Create a point 1 unit in front of origin for a given frame
        point = PointStamped()
        point.header.frame_id = self._frame
        point.header.stamp = rclpy.time.Time().to_msg()
        point.point.x = 1.0
        point.point.y = 0.0
        point.point.z = 0.0

        # Transform to world coordinates and print to console
        try:
            tf_point = self._buffer.transform(point, "world")
            x = tf_point.point.x
            y = tf_point.point.y
            msg = f"Projected point in world frame: ({x:.2f}, {y:.2f})"
            self.get_logger().info(msg)
        except Exception as e:
            self.get_logger().warn(f"Transform failed: {e}")

def main():
    try:
        rclpy.init()
        node = PointProjector("turtle1")
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass

if __name__ == '__main__':
    main()
