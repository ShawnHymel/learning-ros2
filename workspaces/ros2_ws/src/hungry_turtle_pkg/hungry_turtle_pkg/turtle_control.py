#!/usr/bin/env python3
import math

from hungry_turtle_interfaces.msg import TurtleArray, TurtleInfo
from geometry_msgs.msg import Twist
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from turtlesim.msg import Pose

# Debug flag
DEBUG = False

class TurtleControl(Node):
    """Node for controlling the main turtle"""

    def __init__(self, node_name, **kwargs):

        # Initialize parent class
        super().__init__(node_name, **kwargs)

        # Set up parameters
        self.declare_parameter("timer_period", 0.2)
        self.declare_parameter("lin_vel_default", 0.0)
        self.declare_parameter("lin_vel_multiplier", 0.5)
        self.declare_parameter("lin_vel_min", 0.2)
        self.declare_parameter("lin_vel_max", 3.0)
        self.declare_parameter("ang_vel_default", 0.0)
        self.declare_parameter("ang_vel_multiplier", 2.0)
        self.declare_parameter("ang_vel_min", -4.0)
        self.declare_parameter("ang_vel_max", 4.0)

        # Assign parameters
        self._timer_period = self.get_parameter("timer_period").value
        self._player_lin_vel= \
            self.get_parameter("lin_vel_default").value
        self._player_ang_vel = \
            self.get_parameter("ang_vel_default").value

        # Initialize player position subscriber
        self._player_x = 0.0
        self._player_y = 0.0
        self._player_theta = 0.0
        self._turtle_pos_sub = self.create_subscription(
            Pose,
            "/turtle1/pose",
            self._turtle_pos_callback,
            10,
        )

        # Initialize turtles subscriber
        self._turtles = []
        self._closest_turtle = {}
        self._turtles_sub = self.create_subscription(
            TurtleArray,
            "/turtle_spawns",
            self._turtle_spawn_callback,
            10,
        )

        # Configure control publisher
        self._pub = self.create_publisher(
            Twist,
            "/turtle1/cmd_vel",
            10,
        )

        # Configure periodic control updates
        self._timer = self.create_timer(
            self._timer_period, 
            self._control_callback
        )

        # Log that node has started
        self.get_logger().info(node_name + " has started")

    def _control_callback(self):
        """Publish control values for the main turtle"""

        # If there are no turtles, just do some donuts
        if self._turtles == []:
            self._player_lin_vel = \
                self.get_parameter("lin_vel_default").value
            self._player_ang_vel = \
                self.get_parameter("ang_vel_default").value
            
        # Otherwise, angle the turtle toward the nearest spawn
        else:

            # Position
            vx = self._closest_turtle["x"] - self._player_x
            vy = self._closest_turtle["y"] - self._player_y
            dist = math.sqrt(vx ** 2 + vy ** 2)
            self._player_lin_vel = 2 * dist

            # Orientation
            phi = math.atan2(vy, vx)
            ang_diff = phi - self._player_theta
            if ang_diff > math.pi:
                ang_diff -= 2 * math.pi
            elif ang_diff < -math.pi:
                ang_diff += 2 * math.pi
            self._player_ang_vel = 6 * ang_diff

            # # Figure out which way we need to turn using the cross product
            # dx = math.cos(self._player_theta)
            # dy = math.sin(self._player_theta)
            # vx = self._closest_turtle["x"] - self._player_x
            # vy = self._closest_turtle["y"] - self._player_y
            # cross = (dx * vy) - (dy * vx)
            # if DEBUG:
            #     self.get_logger().info(f"cross={cross:.2f}")

            # # Get distance to target
            # dist = math.sqrt(vx ** 2 + vy ** 2)
            # if DEBUG:
            #     self.get_logger().info(f"dist={dist:.2f}")

            # # Update turtle linear velocity
            # lin_vel_mult = self.get_parameter("lin_vel_multiplier").value
            # lin_vel_min = self.get_parameter("lin_vel_min").value
            # lin_vel_max = self.get_parameter("lin_vel_max").value
            # lin_vel = abs(lin_vel_mult / max(cross, 0.1)) * dist
            # self._player_lin_vel = min(max(lin_vel, lin_vel_min), lin_vel_max)
            # if DEBUG:
            #     self.get_logger().info(f"lin_vel={self._player_lin_vel:.2f}")

            # # Update turtle angular velocity
            # ang_vel_mult = self.get_parameter("ang_vel_multiplier").value
            # ang_vel_min = self.get_parameter("ang_vel_min").value
            # ang_vel_max = self.get_parameter("ang_vel_max").value
            # self._player_ang_vel = \
            #     min(max(ang_vel_mult * cross, ang_vel_min), ang_vel_max)
            # if DEBUG:
            #     self.get_logger().info(f"ang_vel={self._player_ang_vel:.2f}")
            #     self.get_logger().info("---")

        # Construct message
        msg = Twist()
        msg.linear.x = self._player_lin_vel
        msg.angular.z = self._player_ang_vel
        self._pub.publish(msg)

    def _turtle_pos_callback(self, msg: Pose):
        """Update player turtle position"""
        self._player_x = msg.x
        self._player_y = msg.y
        self._player_theta = msg.theta

    def _turtle_spawn_callback(self, msg: TurtleArray):
        """Receive list of spawned turtles"""

        # Clear and recreate turtle list
        self._turtles = []
        for turtle in msg.turtles:
            self._turtles.append({
                "name": turtle.name,
                "x": turtle.x,
                "y": turtle.y,
                "theta": turtle.theta,
            })

        # Print turtles
        for turtle in self._turtles:
            self.get_logger().info(f"{turtle['name']}: x={turtle['x']} " +
                                   f"y={turtle['y']} theta={turtle['theta']}")
            
        # Find closest turtle
        self._closest_turtle = {}
        closest_dist = math.inf
        for turtle in self._turtles:
            dist = math.sqrt((self._player_x - turtle["x"])**2 + 
                             (self._player_y - turtle["y"])**2)
            if dist < closest_dist:
                self._closest_turtle = turtle
                closest_dist = dist

def main():
    """Entrypoint"""
    try:
        rclpy.init()
        node = TurtleControl('turtle_control')
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass

if __name__ == '__main__':
    main()