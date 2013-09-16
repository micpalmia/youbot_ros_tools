youbot_ros_tools
================

A [KUKA youBot][1] description package for ROS Hydro to be used for simulation in Gazebo 1.9.

## What's inside
*youbot_ros_tools* is a [catkin][2] package and is divided in four parts, each of which groups a specific set of functionalities. Its structure reflects the one proposed in the Gazebo [ROS integration guidelines][3].

* **youbot_description** contains the Xacro URDF description of the robot, including the directives for Gazebo simulation, and the 3d meshes for all its parts. 
* **youbot_gazebo** contains the launch files for different youbot configurations and world files.
* **youbot_control** provides configuration and launch files to spawn the robot controllers.
* **youbot_teleop** includes a simple teleoperation script for moving the robot's base and arm. More about controlling multiple robots can be found later in the document.

Even though this package is still under developement and contains some tradeoffs that limit its adherence to reality, we encourage you to try it, suggest improvements and raise issues.

## Quick start
After downloading and building the package, if you're in a rush to see a little youBot inside Gazebo and to move its arm, try

```
roslaunch youbot_gazebo youbot.launch
```
to spawn a single youbot in an empty world.
To let the robot start publishing the joint states and to be able to control it, in another terminal type

```
roslaunch youbot_control youbot_control.launch
```
If everything went as expected and neither screens showed errors, you can start the teleoperation script typing

```
roslaunch youbot_teleop youbot_teleop.launch
```
Please type `help` for the list of available commands, or simply try `base_move` to test if the base correctly moves forward and `arm_move j1 3.15` to test if the first joints rotate correctly.

Every one of the precedent commands allows a namespace-specific variation in case multiple robots have been spawned

```
roslaunch youbot_gazebo youbot_double.launch

roslaunch youbot_control youbot_control.launch ns:=/youbot0
roslaunch youbot_control youbot_control.launch ns:=/youbot1

roslaunch youbot_teleop youbot_teleop.launch ns:=/youbot0
roslaunch youbot_teleop youbot_teleop.launch ns:=/youbot1

```
The wiki page about [understanding launch and namespaces][6] gives more insight in this direction.

## Credits
The Xacro URDF description and the meshes for the robot have been taken from the WPI's [youbot_description][4] package. A lot of the code for this repository has been written by [@Arn-O][5].

[1]: http://www.youbot-store.com/
[2]: http://wiki.ros.org/catkin
[3]: http://gazebosim.org/wiki/Tutorials#ROS_Integration
[4]: https://github.com/WPI-RAIL/youbot_description
[5]: https://github.com/Arn-O
[6]: https://github.com/Boanerghes/youbot_ros_tools/wiki/Understanding-launch-and-namespaces
