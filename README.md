# AURO – Autonomous Multi-Robot System (ROS2)

This project implements an **autonomous robotic system** using **ROS2** to simulate and evaluate **item detection**, **navigation**, and **delivery** in dynamic environments.  
It supports **single and multi-robot configurations**, **random seed variations**, and **zone deactivations**, allowing flexible and scalable testing scenarios within a Gazebo simulation environment.

---

## Project Overview

**Objective:**  
Develop an autonomous system capable of detecting, retrieving, and sorting items in a simulated environment using **ROS2** and **TurtleBot3 Waffle Pi** robots.

**Core Features:**
- Multi-robot collaboration (up to 3 robots)
- Camera-based item detection and color-based sorting
- Autonomous navigation via ROS2 **Nav2** stack
- Dynamic scenario handling (zone deactivation, odometry source change)
- Deterministic state-machine-based control
- Full modular ROS2 node architecture (perception, control, navigation, simulation)

---

## System Architecture

### Key Components
| Node | Function |
|------|-----------|
| **Item Sensor** | Detects items and publishes their attributes (`/robot_name/items`) |
| **Robot Sensor** | Detects other robots and publishes their positions (`/robot_name/robots`) |
| **Zone Sensor** | Detects active zones and publishes attributes (`/robot_name/zone`) |
| **Robot Controller** | Central node managing navigation, pickup, and offload actions |
| **Nav2 Stack** | Provides autonomous path planning and navigation |
| **ItemManager** | Simulates the environment, manages item locations and service calls |

---

## Setup Instructions

### 1️⃣ Clone the Repository
Clone this repository inside the `src` folder of your ROS2 workspace:
```bash
cd ~/ros2_ws/src
git clone https://github.com/Tunasisman/Autonomous-Robotic-Systems-Project-Multi-Robot.git
```
### 2️⃣ Build the Workspace
```bash
cd ~/ros2_ws
colcon build
source install/setup.bash
```
### 3️⃣ Launch the Simulation
Start the system with:
```bash
ros2 launch solution solution_nav2_launch.py
```
## Running the Scenarios

Below are the available simulation scenarios and their corresponding commands:

| # | Scenario | Description | Command |
|---|-----------|--------------|----------|
| 1 | **Single Robot** | Launch the simulation with one robot. | ```bash<br>ros2 launch solution solution_nav2_launch.py<br>``` |
| 2 | **Two Robots** | Launch the simulation with two robots. | ```bash<br>ros2 launch solution solution_nav2_launch.py num_robots:=2<br>``` |
| 3 | **Three Robots** | Launch the simulation with three robots. | ```bash<br>ros2 launch solution solution_nav2_launch.py num_robots:=3<br>``` |
| 4 | **Random Seed Variation** | Run with a specific random seed to vary item placement. | ```bash<br>ros2 launch solution solution_nav2_launch.py random_seed:=3<br>``` |
| 5 | **Single Zone Deactivation** | Deactivate the bottom-left zone. | ```bash<br>ros2 launch solution solution_nav2_launch.py zone_bottom_left:=false<br>``` |
| 6 | **Two Zone Deactivation (Two Robots)** | Deactivate the bottom-left and top-left zones while using two robots. | ```bash<br>ros2 launch solution solution_nav2_launch.py zone_bottom_left:=false zone_top_left:=false num_robots:=2<br>``` |
| 7 | **Odometry Source Variation** | Change the odometry source to world-based. | ```bash<br>ros2 launch solution solution_nav2_launch.py odometry:=world<br>``` |

> **Tip:** Always run `source install/setup.bash` in your workspace before launching any scenario to ensure all nodes and parameters are properly recognized.
## Repository Structure
```bash
AURO/
├── launch/
│   └── solution_nav2_launch.py        # Launch configuration
├── src/
│   ├── item_sensor.py                 # Item detection node
│   ├── robot_sensor.py                # Robot detection node
│   ├── zone_sensor.py                 # Zone detection node
│   ├── robot_controller.py            # Main control node (state machine)
│   ├── item_manager.py                # Simulation node
│   └── ...
├── worlds/                            # Gazebo world files
├── models/                            # Custom item/zone models
├── README.md
└── LICENSE
```
## Evaluation Summary

The AURO system was tested in multiple configurations (single-, dual-, and triple-robot) over a 120-second simulation window.  
The table below summarizes the key performance metrics:

| Setup | Avg. Task Time (s) | Items Delivered | Total Value | Pick-Up Rate (%) | Delivery Rate (%) | Collisions |
|:------|:------------------:|:----------------:|:------------:|:----------------:|:-----------------:|:-----------:|
| **Single Robot** | 24.0 | 5 | 25 | 100 | 100 | 0 |
| **Two Robots** | 12.0 | 10 | 75 | 100 | 100 | 0 |
| **Three Robots** | 7.5 | 16 | 165 | 100 | 100 | 0 |

### Observations
- **Efficiency Gains:** Throughput and task completion time improved nearly **3×** when scaling from one to three robots.  
- **Collision Avoidance:** Zero collisions recorded in all configurations — strong validation of the Nav2 and perception integration.  
- **Reliability:** 100% success rates for both pick-up and delivery operations across all test cases.  
- **Scalability:** System maintained stability and synchronization as robot count increased.  

### Extended Evaluation
- **Random Seed Variation:** Consistent performance under different item placements, demonstrating environmental adaptability.  
- **Zone Deactivation Tests:** Robots successfully redirected tasks when one or more zones were disabled.  
- **Odometry Source Variation:** Stable navigation under both encoder- and world-based odometry sources.  

> *Result:* The AURO framework delivers reliable, scalable, and efficient multi-robot coordination suitable for warehouse automation and dynamic task environments.
## Future Work
- Dynamic task reassignment when zones deactivate
- Integration of LiDAR-based SLAM for real-world testing
- Improved task scheduling for heterogeneous robot fleets
