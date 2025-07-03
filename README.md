# Digital Twin Repository

This repository contains work from two academic projects focused on autonomous robotics using ROS2.

## Project Overview

### Project Work 2: Digital Twin for Autonomous Robots using ROS2
The `model/` folder contains the implementation and URDF models from the project work focused on creating digital twins for autonomous robots.

### Bachelor Thesis: Autonomous Robot for Plant Management and Energy Efficiency on Photovoltaic Green Rooftops
The `robogardener/` and `nodes/` folders contain experimental test implementations that were attempts to improve and extend the model from Project Work 2 by integrating advanced ROS2 capabilities including Nav2, SLAM Toolbox, ros2_control, and ros2_controllers. However, these implementations were discontinued due to time pressure and remain incomplete with open TODOs.

## Folder Structure

###  `model/`
Contains the batmobile robot digital twin implementation:
- **URDF files**: Robot description and configuration files (`.xacro`, `.urdf`)
- **Launch files**: ROS2 launch scripts for simulation and visualization
- **Configuration**: RViz and control parameter files
- **Simulation**: Gazebo integration and motor control setup

**Key Features:**
- Complete robot model with physics simulation
- RViz visualization support
- Motor control integration
- Gazebo simulation environment

### `robogardener/`
Experimental package for extending the robot model with advanced capabilities:
- **Robot Description**: Extended URDF/XACRO files with additional sensors and actuators
- **Launch Scripts**: ROS2 launch files for integrated system startup
- **Configuration**: Attempted integration of Nav2 and SLAM parameters
- **World Files**: Test simulation environments (`.sdf` files)
- **Examples**: Incomplete implementation nodes

**Attempted Features (Incomplete):**
- Nav2 navigation stack integration
- SLAM Toolbox for mapping and localization
- ros2_control/ros2_controllers framework
- Advanced autonomous navigation
- **Status**: Discontinued due to time constraints - contains open TODOs

### `nodes/`
Supporting ROS2 nodes for the experimental robogardener system:
- **State Publisher**: Basic robot state management and publishing
- **Package Configuration**: ROS2 package setup with extended dependencies
- **Launch Integration**: Incomplete support files for launch configurations
- **Status**: Part of discontinued experimental work

## Getting Started

### Model (Digital Twin)
```bash
# Launch the batmobile simulation
ros2 launch model model_motor.launch.py

# Test robot movement
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 250.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.5}}" --once
```

### Robogardener (Experimental - Incomplete)
```bash
# Note: These launch files may not work as intended due to incomplete implementation
ros2 launch robogardener robogardener.py
```
**⚠️ Warning**: The robogardener implementation is incomplete and have unresolved dependencies and configuration issues.

## Dependencies
- ROS2 (Humble or later)
- Gazebo
- RViz2
- Python 3.x
- Various ROS2 packages (see individual `package.xml` files)

**Additional dependencies attempted in experimental folders:**
- Nav2 navigation stack
- SLAM Toolbox
- ros2_control & ros2_controllers
- *Note: Some dependencies may be missing or incorrectly configured*

## Authors
- Noirin Graham (grahanoi@students.zhaw.ch)

## License
Apache-2.0

---

*This repository represents academic work conducted at ZHAW focusing on autonomous robotics, digital twins, and sustainable agriculture applications.*
