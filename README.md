youbot_ros_tools
================

Use the hydro-revel for more stable and tested code.

## Goals

+ use the kinematics and dynamics specs from [youbot-store][1].
+ use the last meshes also
+ fix the "joint issue" (joints are moving too slowly)
+ improve readability by keeping the official parameters in parameter files

## Quick start

Compatible with ROS Hydro and Gazebo 1.9.

### base only

Launch display in rviz:

```
roslaunch youbot_description youbot_base_only.launch
```

Launch display in Gazebo:

```
roslaunch youbot_gazebo youbot_base_only.launch
```

[1]: http://www.youbot-store.com/youbot-developers/software/simulation/kuka-youbot-kinematics-dynamics-and-3d-model
