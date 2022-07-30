# ArcWeldBot

## Development Environment
- Operating system:     Ubuntu 20.04
- ROS version:          Noetic
- MoveIt version:       MoveIt 1 Noetic
- Dependencies:         All dependencies needed to use ROS and MoveIt only

## Repository structure:
| File/Folder Name | Description |
| ---------------- | ----------- |
| doc | A folder which contains all documentation|
| ur_descriptio_pkg | A packgae which includes the UR10 robot arm description files |
| ur_moveit_config | An auto-generated package from MoveIt! Setup Assistant to configure the UR10 robot arm |
| LICENSE | GNU GPL v3.0 |
| README.md | This file! |

## Building the project:
Simply clone this repositry inside the source directory of your MoveIt! workspace and then build it using `catkin build`:
        
        cd path/to/moveit_ws/src/ArcWeldBot
        git clone https://github.com/FShahin91/ArcWeldBot.git
        cd ../
        catkin build