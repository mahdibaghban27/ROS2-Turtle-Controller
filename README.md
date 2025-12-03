#  ROS2 Turtle Controller — UI & Distance Nodes  
A ROS2 project developed using **Docker**, **VS Code Remote Containers**, and **XLaunch** to control two turtles in `turtlesim` with collision & boundary monitoring.

![Demo](assignment1_rt/assets/rt1_demo.gif)

## 📌 Overview

This project implements **two ROS2 nodes** inside a custom package:

### 1️⃣ UI Node (`ui_node`)
A simple terminal-based interface that:
- Selects turtle (`turtle1` or `turtle2`)
- Accepts linear & angular velocity
- Sends command for 1 second
- Stops turtle automatically
- Repeats input loop
- Publishes active turtle

### 2️⃣ Distance Node (`distance_node`)
Monitors:
- Distance between turtles (`turtles_distance`)
- Stops moving turtle if too close
- Stops turtle near boundaries
- Cooldown system to avoid lock
- Only stops linear motion on walls (prevents sticking)

## 🛠️ Technologies Used
- ROS2 Humble  
- C++  
- Docker  
- VS Code Remote Containers  
- XLaunch  
- turtlesim  

## 🐳 Running the Project (Step-by-Step)

> ⚠️ **Before anything:**  
> Start XLaunch → Multiple Windows → Start no client → Disable access control  

### 1️⃣ Start turtlesim
```bash
cd ~/ros2_ws
source install/setup.bash
ros2 run turtlesim turtlesim_node
```

### 2️⃣ Spawn turtle2
```bash
ros2 service call /spawn turtlesim/srv/Spawn "{x: 5.5, y: 5.5, theta: 0.0}"
```

### 3️⃣ Move turtle2 to bottom-left
```bash
ros2 service call /turtle2/teleport_absolute turtlesim/srv/TeleportAbsolute "{x: 2.0, y: 2.0, theta: 0.0}"
```

### 4️⃣ Run Distance Node
```bash
ros2 run assignment1_rt distance_node
```

### 5️⃣ Run UI Node
```bash
ros2 run assignment1_rt ui_node
```

## 📂 Project Structure
```
ros2_ws/
└── src/
    └── assignment1_rt/
        ├── package.xml
        ├── CMakeLists.txt
        ├── src/
        │   ├── ui_node.cpp
        │   └── distance_node.cpp
        └── assets/
```

## 🧠 Node Descriptions

### ⭐ UI Node
- Input loop  
- Velocity commands  
- Publishes active turtle  

### ⭐ Distance Node
- Distance calc  
- Collision prevention  
- Boundary check  
- Cooldown logic  

## ✔️ Final Result
A fully interactive ROS2 system that:
- Controls two turtles  
- Avoids collisions  
- Handles boundaries  
- Runs well under Docker + VS Code  

## 👤 Author
**Mahdi Baghban**  
GitHub: https://github.com/mahdibaghban27
