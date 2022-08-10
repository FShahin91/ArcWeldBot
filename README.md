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
| scene_and_ctrl | A package which includes nodes to handle populating the scene and control the robot|
| ur_description_pkg | A package which includes the UR10 robot arm description files |
| ur_moveit_config | An auto-generated package from MoveIt! Setup Assistant to configure the UR10 robot arm |
| LICENSE | GNU GPL v3.0 |
| README.md | This file! |

## Building the project:
Simply clone this to your workspace and then build it (for example uising `catkin build`):
        
        cd path/to/workspace/src/
        git clone https://github.com/FShahin91/ArcWeldBot.git
        cd ../
        catkin build
