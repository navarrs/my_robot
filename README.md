# Project 2 - Go Chase It

Simple Gazebo and Simulation of a object follower robot. 

To compile:
```
	cd /path/to/catkin_ws/src/
	catkin_init_workspace
	cd ..
	catkin_make
```

## Launch World 
To launch the world simulation (Gazebo and RVIZ). In one terminal, run:
```
	cd /path/to/catkin_ws/
	source devel/setup.zsh
	roslaunch my_robot world.launch
```

Gazebo should look like this:
<p align="center">
    <img src="./readme/gazebo.png" width="800" />
</p>

Rviz should look like this:
<p align="center">
   <img src="./readme/rviz.png" width="800" />
</p>

## Launch Ball Chaser

To launch the ball chaser. In another terminal, run:
```
	cd /path/to/catkin_ws/
	source devel/setup.zsh
	roslaunch ball_chaser ball_chaser.launch
```

## Visualize raw image 
To visualize the images from the robot camera, run:
```
	cd /path/to/catkin_ws/
	source devel/setup.zsh
	rosrun rqt_image_view rqt_image_view
```

Make sure to select the topic ```/camera/rgb/image_raw/```. And it should look like this:
<p align="center">
   <img src="./readme/image.png" width="600" />
</p>


Now you can start moving the white ball in front of the robot in Gazebo. It should start moving left, right or forward, depending on where the ball is. If the robot does not see the ball, it should not move. 