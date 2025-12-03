ROS2 Turtle Controller — UI & Distance Nodes

A ROS2 project developed using Docker, VS Code Remote Containers, and XLaunch to control two turtles in turtlesim with collision and boundary monitoring.


![Demo](src/assignment1_rt/asset/rt1_demo.gif)

Overview

This project implements two ROS2 nodes inside a custom package:

1. UI Node (ui_node)

A simple terminal-based interface that:

Selects turtle (turtle1 or turtle2)

Accepts linear and angular velocity

Sends command for 1 second

Automatically stops turtle

Repeats the input loop

Publishes which turtle is active

2. Distance Node (distance_node)

Monitors:

Distance between turtles (turtles_distance)

Stops moving turtle if distance is too small

Stops turtle when approaching boundaries

Includes cooldown system to avoid repeated stopping

Stops only linear motion near walls to prevent sticking

Technologies Used

ROS2 Humble

C++

Docker

VS Code Remote Containers

XLaunch

turtlesim

Running the Project (Step-by-Step)

(Ensure you start XLaunch → Multiple Windows → Start no client → Disable access control)

1. Start turtlesim
cd ~/ros2_ws
source install/setup.bash
ros2 run turtlesim turtlesim_node

2. Spawn turtle2
ros2 service call /spawn turtlesim/srv/Spawn "{x: 5.5, y: 5.5, theta: 0.0}"

3. Move turtle2 to bottom-left
ros2 service call /turtle2/teleport_absolute turtlesim/srv/TeleportAbsolute "{x: 2.0, y: 2.0, theta: 0.0}"

4. Run Distance Node
ros2 run assignment1_rt distance_node

5. Run UI Node
ros2 run assignment1_rt ui_node

Project Structure
ros2_ws/
└── src/
    └── assignment1_rt/
        ├── package.xml
        ├── CMakeLists.txt
        ├── src/
        │   ├── ui_node.cpp
        │   └── distance_node.cpp
        └── assets/

Node Descriptions
UI Node

User input loop

Velocity command publishing

Publishes active turtle

Distance Node

Computes distance

Collision prevention

Boundary checking

Cooldown logic

Final Result

A fully interactive ROS2 system that:

Controls two turtles

Avoids collisions

Handles boundaries

Runs smoothly under Docker and VS Code Remote Containers

Author

Mahdi Baghban
GitHub: https://github.com/mahdibaghban27
