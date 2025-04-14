#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import math

class DynamicFollower(Node):
    def __init__(self):
        super().__init__('dynamic_follower')
        # Parameters: number of turtles and initial leader (default 'turtle1')
        self.declare_parameter('num_turtles', 2)
        self.declare_parameter('initial_leader', 'turtle1')
        self.num_turtles = self.get_parameter('num_turtles').value
        self.leader = self.get_parameter('initial_leader').value

        # Dictionary to store turtle poses by name
        self.poses = {}
        # Dictionary of publishers for each turtle's cmd_vel topic
        self.cmd_pubs = {}

        # Leader switch topic
        self.create_subscription(
            String,
            '/switch_leader',
            self.leader_switch_cb,
            10
        )

        # For each turtle (turtle1 through turtleN), subscribe to pose and create a publisher for cmd_vel
        for i in range(1, self.num_turtles + 1):
            turtle_name = f'turtle{i}'
            self.create_subscription(
                Pose,
                f'/{turtle_name}/pose',
                self._create_pose_cb(turtle_name),
                10
            )
            self.cmd_pubs[turtle_name] = self.create_publisher(
                Twist,
                f'/{turtle_name}/cmd_vel',
                10
            )

        # Special teleop relay: subscribe to /leader/cmd_vel and forward to the current leader
        self.create_subscription(Twist, '/leader/cmd_vel', self.teleop_cmd_cb, 10)
        # Timer for control loop: have followers track the leader
        self.timer = self.create_timer(0.1, self.control_loop)
        self.get_logger().info(f"Initial leader set to: {self.leader}")

    def _create_pose_cb(self, turtle_name):
        # Create a callback that captures the turtle name.
        def cb(msg):
            self.poses[turtle_name] = msg
        return cb

    def leader_switch_cb(self, msg):
        # Topic callback for switching the leader.
        leader_name = msg.data
        if leader_name in self.poses:
            self.leader = leader_name
            self.get_logger().info(f"Leader switched to: {self.leader}")
        else:
            self.get_logger().error(f"Leader {leader_name} does not exist or is not available yet!")

    def teleop_cmd_cb(self, msg):
        # Relay teleop commands to the current leader.
        if self.leader in self.cmd_pubs:
            self.cmd_pubs[self.leader].publish(msg)

    def control_loop(self):
        # Only run if the leader's pose is available.
        if self.leader not in self.poses:
            return

        leader_pose = self.poses[self.leader]
        # Check if leader is moving
        velocity_threshold = 0.001
        leader_moving = (
            abs(leader_pose.linear_velocity) > velocity_threshold or 
            abs(leader_pose.angular_velocity) > velocity_threshold
        )

        # For every turtle (except the leader), compute velocity commands
        for turtle_name, pose in self.poses.items():
            if turtle_name == self.leader:
                continue

            if leader_moving:
                dx = leader_pose.x - pose.x
                dy = leader_pose.y - pose.y
                distance = math.hypot(dx, dy)
                target_angle = math.atan2(dy, dx)
                angle_error = math.atan2(
                    math.sin(target_angle - pose.theta),
                    math.cos(target_angle - pose.theta)
                )

                K_linear = 0.8
                K_angular = 1.2

                cmd = Twist()
                cmd.linear.x = K_linear * distance
                cmd.angular.z = K_angular * angle_error
            else:
                cmd = Twist()
                cmd.linear.x = 0.0
                cmd.angular.z = 0.0

            self.cmd_pubs[turtle_name].publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = DynamicFollower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()