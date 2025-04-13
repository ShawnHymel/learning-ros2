#!/usr/bin/env python3

import math
import random
import uuid

from hungry_turtle_interfaces.msg import TurtleArray, TurtleInfo
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from turtlesim.msg import Pose
from turtlesim.srv import Kill, Spawn

# Debug flag
DEBUG = False
SOUND = True

class GameLogic(Node):
    """Node for spawning new turtles and handling collisions"""

    def __init__(self, node_name, **kwargs):

        # Initialize parent class
        super().__init__(node_name, **kwargs)

        # Set up parameters
        self.declare_parameter("seed", 42)
        self.declare_parameter("spawn_period", 1.0)
        self.declare_parameter("canvas_width", 11.088)
        self.declare_parameter("canvas_height", 11.088)
        self.declare_parameter("max_spawns", 10)
        self.declare_parameter("collision_distance", 0.3)

        # Assign parameters
        random.seed(self.get_parameter("seed").value)
        self._timer_period = self.get_parameter("spawn_period").value
        self._canvas_width = self.get_parameter("canvas_width").value
        self._canvas_height = self.get_parameter("canvas_height").value
        self._max_spawns = self.get_parameter("max_spawns").value

        # Initialize turtle list
        self._turtles = []

        # Configure turtle list publisher
        self._turtles_pub = self.create_publisher(
            TurtleArray,
            "/turtle_spawns",
            10,
        )

        # Configure clients
        self._spawn_client = self.create_client(Spawn, "/spawn")
        self._kill_client = self.create_client(Kill, "/kill")

        # Configure periodic control updates
        self._timer = self.create_timer(
            self._timer_period, 
            self._spawn_turtle
        )

        # Initialize player position subscriber
        self._player_sub = self.create_subscription(
            Pose,
            "/turtle1/pose",
            self._game_logic_callback,
            10,
        )

        # Log that node has started
        self.get_logger().info(node_name + " has started")

    def _spawn_turtle(self):
        """Send a request to spawn a new turtle in a random position"""
        
        # Check to see if we've reached the maximum number of turtles
        if len(self._turtles) >= self._max_spawns:
            self.get_logger().info("Cannot spawn turtle. Max of " + 
                                   str(self._max_spawns) + 
                                   " reached")

        # Otherwise, spawn away!
        else: 

            # Say that we're spawning
            self.get_logger().info("Spawning turtle " + 
                                   str(len(self._turtles) + 1) +
                                   " of " +
                                   str(self._max_spawns))

            # Generate new name, spawn location, and heading
            name = "trtl_" + str(uuid.uuid4())[:8]
            x = random.random() * self._canvas_width
            y = random.random() * self._canvas_height
            theta = random.random() * 2 * math.pi
            self._turtles.append({
                "name": name,
                "x": x,
                "y": y,
                "theta": theta,
            })

            # Construct request
            req = Spawn.Request()
            req.name = name
            req.x = x
            req.y = y
            req.theta = theta

            # Send request and set callback
            future = self._spawn_client.call_async(req)
            future.add_done_callback(self._spawn_callback)

        # Construct a message list of all spawned turtles
        msg = TurtleArray()
        for turtle in self._turtles:
            info = TurtleInfo()
            info.name = turtle["name"]
            info.x = turtle["x"]
            info.y = turtle["y"]
            info.theta = turtle["theta"]
            msg.turtles.append(info)
        
        # Publish list of all spawned turtles
        self._turtles_pub.publish(msg)

    def _spawn_callback(self, future):
        """Called on spawn response from turtlesim server"""

        # Print turtle name
        resp = future.result()
        self.get_logger().info("Spawn response: " + str(resp))

    def _game_logic_callback(self, msg: Pose):
        """On player position update, look for collisions"""

        # Get player position
        player_x = msg.x
        player_y = msg.y

        # Check for collisions
        col_dist = self.get_parameter("collision_distance").value
        for turtle in self._turtles:
            dist = math.sqrt((player_x - turtle["x"]) ** 2 + \
                (player_y - turtle["y"]) ** 2)
            if DEBUG:
                self.get_logger().info(
                    f"Distance from {turtle["name"]}: {dist}"
                )
            
            # Collision detected
            if dist <= col_dist:
                self.get_logger().info(f"Collision with {turtle['name']}")

                # Send request to remove turtle
                req = Kill.Request()
                req.name = turtle["name"]
                self._kill_client.call_async(req)

                # Update turtles list
                self._turtles[:] = [t for t in self._turtles if t["name"] != \
                                    turtle["name"]]
                
                break

def main():
    try:
        rclpy.init()
        node = GameLogic('game_logic')
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass

if __name__ == '__main__':
    main()