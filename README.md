# IRS_2025_Group_4

All laboratory work produced by Group 4 throughout Industrial Robots and Systems (12059), Semester 2 2025.

## Description

IRS_2025_Group_4 is a directory containing all labortaory work from Week 5 onwards. This include the introduction of subscriber and publisher nodes, robot mapping, localisation, and path planning as well as  robot arm kinematics and motion planning. These concepts were intergrateed to develop a program that autonomously controls the OMRON robot to perform a continuous loop: navigating to the converyor belt, picking up the detected box, and placing on an empty shelf. It was built using ROS2, Nav2, and MoveIt.

## Getting Started

### Dependencies
This package was developed and tested on **Ubuntu 22.04** with **ROS2 Humble**.

#### Runtime Dependencies
The following ROS2 packages are required to run the project:
* `ros2launch` - for launching nodes and systems  
* `slam_toolbox` - for mapping and localisation  
* `rviz2` - for visualisation  
* `tf2_ros` - for coordinate frame transformations  
* `rclpy` - Python client library for ROS2 nodes  
* `action_msgs` - standard ROS2 action message definitions  
* `moveit_msgs` - messages for motion planning with MoveIt  
* `trajectory_msgs` - message definitions for robot trajectories 


### Installing

* Create a `src` directory and clone the repository: https://github.com/iamamiina/IRS_2025_Group_4.git 
* Install dependencies:
`sudo apt install ros-humble-ros2launch ros-humble-slam-toolbox ros-humble-rviz2 ros-humble-tf2-ros ros-humble-rclpy ros-humble-action-msgs ros-humble-moveit-msgs ros-humble-trajectory-msgs`

### Executing program

* Push and pull the latest docker container (for IRS Simulation)
* Open the Openplc runtime UI and upload `timebelt_updated.st` file which can be located in the git repository. 
* Start the PLC
* In the IRS Simulation, press the 'P' key to Start the PLC and conveyor belt.
* press the 'R' key and click 'Autonomous Mode'.
* Navigate the the src directory `cd src`
* In three different command windows:
    * Build software packages: `colcon build`
    * Source environment variables: `source install/local_setup.bash`
    * Launch and run:
        1. MoveIt: `ros2 launch tm12x_moveit_config hand_solo_moveit.launch.py`
        2. Nav2: `ros2 launch autonomous_nav nav_launch.py`
        3. Main Program: `ros2 run autonomous_nav hs_waypoint_follower`


## Authors

Amina Badri
[u3275670@uni.canberra.edu.au]

Phoebe Green
[u3283467@uni.canberra.edu.au]

Zafirah Sarwar
[u3280651@uni.canberra.edu.au]

## Version History

* 0.7
    * The final presented program
* 0.6
    * Adjusting and tuning arm joint positions and map locations
* 0.5
    * Implemenation of loop function
    * Debugging
* 0.4
    * Create new package for final project
    * Start of developing autonomous functionality
    * Moves robot autonomously
    * Moves arm joints autonomously
* 0.3
    * Path planning for final project
* 0.2
    * All the completed laboratory work from  week 5-10
* 0.1
    * Initial Release

## License

This project is licensed under the Apache License 2.0.

## Acknowledgments

Course run by Dr. Maleen Jayasuriya
