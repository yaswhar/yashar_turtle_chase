Sure! Here's the complete `README.md` in Markdown code format:

# 🐢 yashar_turtle_chase

A dynamic leader-follower demo using ROS 2's `turtlesim`, where multiple turtles can follow a designated leader with real-time leader switching capability.

## 📦 Overview

This package demonstrates a multi-turtle simulation where:

- Multiple turtles are spawned in the `turtlesim` environment.
- One turtle acts as the leader, controllable via keyboard.
- Other turtles follow the leader using proportional control.
- The leader can be switched dynamically at runtime.

## 🧰 Features

- **Dynamic Turtle Spawning**: Automatically spawns a specified number of turtles at random, non-overlapping positions.
- **Leader-Follower Mechanism**: Implements a simple proportional controller for followers to track the leader's position.
- **Real-Time Leader Switching**: Change the leader turtle during runtime using a ROS 2 topic.
- **Launch File**: Convenient launch file to start the entire setup, including `turtlesim`, turtle spawner, follower logic, and teleoperation.

## 🚀 Installation

1. **Clone the Repository**:

   ```bash
   cd ~/ros2_ws/src
   git clone https://github.com/your_username/yashar_turtle_chase.git

2. **Install Dependencies**:

   ```bash
   cd ~/ros2_ws
   rosdep install --from-paths src --ignore-src -r -y
   ```

3. **Build the Package**:

   ```bash
   colcon build --packages-select yashar_turtle_chase
   source install/setup.bash
   ```

## 🧪 Usage

### Launch the Simulation

```bash
ros2 launch yashar_turtle_chase dynamic_chase.launch.py
```

This will:

- Start the `turtlesim_node`.
- Spawn the specified number of turtles (default is 2).
- Initiate the dynamic follower node.
- Open a terminal for teleoperation.

### Control the Leader Turtle

Use the keyboard in the teleoperation terminal to move the leader turtle.

### Switch the Leader Turtle

Publish a new leader name to the `/switch_leader` topic:

```bash
ros2 topic pub /switch_leader std_msgs/String "{data: 'turtle2'}"
```

Replace `'turtle2'` with the desired turtle name (e.g., `'turtle1'`, `'turtle3'`, etc.).

## 🧠 Implementation Details

### Follower Logic

The `dynamic_follower.py` node:

- Subscribes to the `/turtleX/pose` topics to get the positions of all turtles.
- Publishes velocity commands to `/turtleY/cmd_vel` for follower turtles.
- Calculates the distance and angle to the leader and applies proportional control to follow.

### Leader Detection

The current leader is stored as a variable within the node. It can be updated at runtime by publishing to the `/switch_leader` topic.

### Challenges Faced

- **Dynamic Spawning**: Ensuring turtles spawn at non-overlapping positions required implementing a check against existing positions.
- **Real-Time Leader Switching**: Maintaining smooth transitions when changing leaders without disrupting follower behavior.

## 📹 Demonstration

![Demo GIF](path_to_demo.gif)

*In the demo, observe how the follower turtle tracks the leader and how switching the leader affects the behavior.*

## 📝 License

This project is licensed under the Apache License 2.0.

## 🙋‍♂️ Author

**Yashar Zafari**

Email: [zafari.h.yashar@gmail.com](mailto:zafari.h.yashar@gmail.com)

---

*For more information on ROS 2 and `turtlesim`, refer to the [official ROS 2 tutorials](https://docs.ros.org/en/foxy/Tutorials/Beginner-CLI-Tools/Introducing-Turtlesim/Introducing-Turtlesim.html).*
