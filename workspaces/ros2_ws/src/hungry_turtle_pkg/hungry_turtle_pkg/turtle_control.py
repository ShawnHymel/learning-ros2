#!/usr/bin/env python3
import math

from hungry_turtle_interfaces.msg import TurtleArray, TurtleInfo
from geometry_msgs.msg import Twist
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from turtlesim.msg import Pose

class TurtleControl(Node):
    """Node for controlling the main turtle"""

    def __init__(self, node_name, **kwargs):

        # Initialize parent class
        super().__init__(node_name, **kwargs)

        # Set up parameters
        self.declare_parameter("timer_period", 0.5)
        self.declare_parameter("default_linear_velocity", 1.0)
        self.declare_parameter("linear_velocity_multiplier", 3.0)
        self.declare_parameter("default_angular_velocity", 1.0)
        self.declare_parameter("angular_velocity_multiplier", 10.0)

        # Assign parameters
        self._timer_period = self.get_parameter("timer_period").value
        self._player_lin_vel= \
            self.get_parameter("default_linear_velocity").value
        self._player_ang_vel = \
            self.get_parameter("default_angular_velocity").value

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
                self.get_parameter("default_linear_velocity").value
            self._player_ang_vel = \
                self.get_parameter("default_angular_velocity").value
            
        # Otherwise, angle the turtle toward the nearest spawn
        else:

            # Figure out which way we need to turn using the cross product
            dx = math.cos(self._player_theta)
            dy = math.sin(self._player_theta)
            vx = self._closest_turtle["x"] - self._player_x
            vy = self._closest_turtle["y"] - self._player_y
            cross = (dx * vy) - (dy * vx)
            self.get_logger().info(f"cross={cross}")

            # Get distance to target
            dist = math.sqrt(vx ** 2 + vy ** 2)
            self.get_logger().info(f"dist={dist}")
            self.get_logger().info("---")

            # Update turtle linear velocity
            lin_vel = \
                self.get_parameter("default_linear_velocity").value
            lin_mult = self.get_parameter("linear_velocity_multiplier").value
            if dist > 1.0:
                self._player_lin_vel = lin_mult * lin_vel
            else:
                self._player_lin_vel = lin_vel

            # Update turtle angular velocity
            ang_mult = self.get_parameter("angular_velocity_multiplier").value
            ang_vel = self.get_parameter("default_angular_velocity").value
            if cross == 0:
                self._player_ang_vel = 0
            elif cross > 0:
                self._player_ang_vel = ang_vel
            else:
                self._player_ang_vel = -1 * ang_vel

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