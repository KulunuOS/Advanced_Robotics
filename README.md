# AUT-720 Advanced Robotics

We'll use [[4]](https://github.com/modulabs/arm-control) for implementing and demonstrating the course assignments.  

### Installation and Pre-requisites
We'll use [[4]](https://github.com/modulabs/arm-control) for implementing and demonstrating the course assignments. 
Follow the installation guideslines and install the dependencies as instructed in mthe original work

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

## References
1. [ros_Control](http://wiki.ros.org/ros_control)
2. [Write a new ros-controller](https://github.com/ros-controls/ros_control/wiki/controller_interface)
3. [Elfin Robot](http://wiki.ros.org/Robots/Elfin)
4. [Elfin Simulation package](https://github.com/modulabs/arm-control)
5. [ROS](http://wiki.ros.org/)

