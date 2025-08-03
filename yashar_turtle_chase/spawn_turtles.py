#!/usr/bin/env python3

# Copyright 2025 Yashar Zafari
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import math
import random

import rclpy
from rclpy.node import Node
from turtlesim.srv import Spawn


class TurtleSpawner(Node):

    def __init__(self):
        super().__init__('turtle_spawner')
        # Parameter for the total number of turtles (default is 2)
        self.declare_parameter('num_turtles', 2)
        self.num_turtles = self.get_parameter('num_turtles').value

        # Spawn Service Client
        self.cli = self.create_client(Spawn, '/spawn')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Spawn service not available, waiting...')
        # List to store positions (x, y) already used.
        # Assume turtle1's default position is (5.544444, 5.544444)
        self.existing_positions = [(5.544444, 5.544444)]
        self.spawn_turtles()

    def generate_random_position(self, min_distance=1.0, x_range=(1.0, 10.0),
                                 y_range=(1.0, 10.0), max_attempts=50):
        """Generate a random (x, y) position not too close to any existing positions."""
        for _ in range(max_attempts):
            x = random.uniform(*x_range)
            y = random.uniform(*y_range)
            is_clear = all(
                math.hypot(x - ex, y - ey) >= min_distance for ex, ey in self.existing_positions)
            if is_clear:
                return x, y
        raise RuntimeError('Could not generate a non-colliding position after many attempts')

    def spawn_turtles(self):
        """Spawn turtles dynamically."""
        # Since turtle1 is automatically spawned by turtlesim_node,
        # spawn turtles from turtle2 up to num_turtles.
        for i in range(2, self.num_turtles + 1):
            req = Spawn.Request()
            x, y = self.generate_random_position()
            req.x = x
            req.y = y
            req.theta = random.uniform(0, 2 * math.pi)
            req.name = f'turtle{i}'
            self.existing_positions.append((x, y))
            future = self.cli.call_async(req)
            future.add_done_callback(
                lambda fut, name=req.name: self.spawn_callback(fut, name))

    def spawn_callback(self, future, turtle_name):
        try:
            response = future.result()
            self.get_logger().info(f'Spawned {response.name} at a random position.')
        except Exception as e:
            self.get_logger().error(f'Failed to spawn {turtle_name}: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = TurtleSpawner()
    # Give a brief period for turtles to spawn.
    rclpy.spin_once(node, timeout_sec=0.5)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
