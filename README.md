# AUT-720 Advanced Robotics


### Installation and Pre-requisites
We'll use [[4]](https://github.com/modulabs/arm-control) for implementing and demonstrating the course assignments. 
Follow the [prerequisites](https://github.com/modulabs/arm-control#prerequisite) and install the dependencies listed in Original repository.

### Download, create a workspace and build 

    $ mkdir -p elfin_ws/src
    $ cd ~/elfin_ws/src
    $ git clone https://github.com/tau-alma/Elfin_Simulation.git
    $ cd ~/elfin_ws/
    $ catkin build

## Test our own implemented controllers

- [x] Kinematic controller

```
$ roslaunch elfin_gazebo elfin3_kinematic_controller.launch controller:=kinematic_controller
``` 

- [x] Inverse Dynamics controller

```
roslaunch elfin_gazebo elfin3_empty_world.launch controller:=dynamic_controller
``` 

## References
1. [ros_Control](http://wiki.ros.org/ros_control)
2. [Write a new ros-controller](https://github.com/ros-controls/ros_control/wiki/controller_interface)
3. [Elfin Robot](http://wiki.ros.org/Robots/Elfin)
4. [Elfin Simulation package](https://github.com/modulabs/arm-control)
5. [ROS](http://wiki.ros.org/)

