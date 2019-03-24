<img src="http://moveit.ros.org/assets/images/moveit2_logo_black.png" alt="MoveIt! Logo" width="200"/>

The MoveIt! Motion Planning Framework **for ROS 2.0**

- [Milestones](#milestones)
- [Overview of MoveIt!](http://moveit.ros.org)
- [Installation instructions for MoveIt 2](https://acutronicrobotics.com/docs/products/robots/mara/moveit2/install)
  - [Ubuntu 18.04](https://acutronicrobotics.com/docs/products/robots/mara/moveit2/install/ubuntu)
  - [OS X 10.14](https://acutronicrobotics.com/docs/products/robots/mara/moveit2/install/osx)
- [Documentation](http://moveit.ros.org/documentation/)
- [Get Involved](http://moveit.ros.org/documentation/contributing/)

## Milestones
- [x] Install instructions
  - [x] [Ubuntu 18.04](https://acutronicrobotics.com/docs/products/robots/mara/moveit2/install/ubuntu)
  - [x] [OS X 10.14](https://acutronicrobotics.com/docs/products/robots/mara/moveit2/install/osx)
- [x] Upgrade continuous integration for ROS 2.0
<details><summary>Update/setup infrastructure for development</summary>

- [x] Update/setup infrastructure for development
  - [x] Delete metapackages
  - [x] Upgrade continuous integration for ROS 2.0
  - [x] Refactor/cleanup folder hierarchy
</details>

- [x] Convert all headers and link it to HRIM (contributed by @ibaiape)
<details><summary>Dependencies on other packages</summary>

- [x] Dependencies on other packages
  - [x] tf2_kdl https://github.com/ros2/geometry2/pull/90
  - [x] eigen_stl_containers https://github.com/AcutronicRobotics/eigen_stl_containers/tree/ros2
  - [x] geometric_shapes https://github.com/ros-planning/geometric_shapes/pull/96
  - [x] random_numbers https://github.com/ros-planning/random_numbers/pull/12
  - [x] srdfdom (contributed by @anasarrak, @vmayoral and @ahcorde) https://github.com/ros-planning/srdfdom/pull/45
  - [x] urdf_parser_py https://github.com/ros/urdf_parser_py/pull/41
  - [x] Created a ROS 2 version (with package.xml) of urdfdom_headers https://github.com/AcutronicRobotics/urdfdom_headers/tree/ros2
  - [x] octomap https://github.com/AcutronicRobotics/octomap
    - [x]  octomap
    - [ ]  octovis
    - [ ]  dynamicEDT3D
</details>

<details><summary>Convert moveit_core packages to ROS 2.0</summary>

- [x] Convert moveit_core packages to ROS 2.0
  - [x] version
  - [x] macros
  - [x] backtrace
  - [x] exceptions
  - [x] profiler
  - [x] logging
  - [x] background_processing
  - [x] kinematics_base
  - [x] controller_manager
  - [x] sensor_manager
  - [x] robot_model
  - [x] transforms
  - [x] robot_state
  - [x] robot_trajectory
  - [x] collision_detection
  - [x] collision_detection_fcl
  - [x] kinematic_constraints
  - [x] planning_scene
  - [x] constraint_samplers
  - [x] planning_interface
  - [x] planning_request_adapter
  - [x] trajectory_processing
  - [x] distance_field
  - [x] collision_distance_field
  - [x] kinematics_metrics
  - [x] dynamics_solver
  - [x] utils
</details>

- [ ] moveit_ros
    - [x] planning_interface
- [ ] Necessary for a Minimal Working Example
    - [ ] moveit_ros_planning_interface
   -  [ ] moveit_ros_planning
       -   [ ] moveit_core
       -   [ ] moveit_ros_perception
   -  [ ] moveit_ros_warehouse
     -  [ ] moveit_ros_planning
     -  [ ] warehouse_ros
   -  [ ] moveit_ros_manipulation
       -   [ ] moveit_core
       -   [ ] moveit_ros_planning
       -   [ ] moveit_ros_move_group
           -   [ ] moveit_core
           -   [ ] moveit_ros_planning
- [ ] New features in ROS 2.0
  - [ ] Migrate plugin architecture to ROS2 nodelets
- [ ] Documentation
  - [ ] Tutorials for MoveIt2
  - [ ] Create tutorial on using ros1/ros2 bridge to support ros1 hardware drivers
  - [ ] Move install instructions to moveit.ros.org

<details><summary>Major refactoring and divergence from moveit2 (<b>not started</b>)</summary>

- [ ] Major refactoring and divergence from moveit2
  - [ ] Run ROS2 C++ and python linters
  - [ ] Delete excesses packages that are left over from rosbuild stacks: moveit_runtime, moveit_plugins, moveit_ros
  - [ ] Rename non-package folders:
    - [ ] rename moveit_planners to planners
    - [ ] rename moveit_plugins to controller_interfaces
  - [ ] Restructure folder layout of moveit repo:
    - [ ] flatten moveit_ros folder to root of repo
    - [ ] rename all moveit_ros folders with moveit_ros prefix
  - [ ] Rename major classes
    - [ ] ControllerManagers become ControllerInterfaces
    - [ ] Rename related packages
  - [ ] Merge repos:
    - [ ] moveit 9.6 MB
    - [ ] moveit_task_constructor
    - [ ] moveit_tutorials  28.6 MB
    - [ ] moveit_msgs
    - [ ] moveit_resources  61 MB
    - [ ] moveit_visual_tools
    - [ ] moveit_advanced?
    - [ ] DELETE: moveit_kinematics_tests
  - [ ] Remove large binaries from moveit repo
  - [ ] Add gitlfs?
</details>

## Continuous Integration Status
[![Build Status](https://travis-ci.org/AcutronicRobotics/moveit2.svg?branch=master)](https://travis-ci.org/AcutronicRobotics/moveit2)

## Docker Containers
TODO [Create ROS2 Docker containers for MoveIt!](https://github.com/ros-planning/moveit2/issues/15)

## ROS Buildfarm
Debian releases of MoveIt2 will not be available during the alpha development stage. Check back May 2019.
