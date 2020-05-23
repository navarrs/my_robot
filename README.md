# Hugo in ROS 

## Requirements
This code is tested in ROS Melodic with Ubuntu 18.04.
```
  sudo apt-get update -y
  sudo apt-get install -y xterm
  sudo apt install ros-melodic-rqt
  sudo apt-get install ros-melodic-navigation
  sudo apt-get install ros-melodic-map-server
  sudo apt-get install ros-melodic-move-base
  sudo apt-get install ros-melodic-amcl
  sudo apt-get install ros-melodic-rtabmap-ros
  sudo apt-get install ros-melodic-teleop-twist-keyboard
```
To compile:
```
  cd /path/to/catkin_ws/src/
  catkin_init_workspace
  cd ..
  catkin_make 
```

## Packages

Our current packages are below. Follow the links for more information on how to use them.
* [Database](https://github.com/navarrs/my_robot/tree/master/catkin_ws/src/database): stores all the assets
* [Localization](https://github.com/navarrs/my_robot/tree/master/catkin_ws/src/localization): used to perform robot self-localization. Currently supported by [RTAB-Map](http://wiki.ros.org/rtabmap_ros) and [AMCL](http://wiki.ros.org/amcl). 
* [Mapping](https://github.com/navarrs/my_robot/tree/master/catkin_ws/src/mapping): Perform environment mapping. Currently supported by RTAB-Map.
* [Object Detector](https://github.com/navarrs/my_robot/tree/master/catkin_ws/src/object_detector): Performs object detection. IN PROGRESS.
* [Robot](https://github.com/navarrs/my_robot/tree/master/catkin_ws/src/robot): Contains the urdf files of all supported robots, as well as their kinematic models. Currently supports two robots, namely: hugo and dummy. 
* [Teleop Key](http://wiki.ros.org/teleop_twist_keyboard): External package to operate the keyboard for robot twists. 

### Notes
 - Make sure to modify executing permissions for the scripts in ```scripts``` folder. 