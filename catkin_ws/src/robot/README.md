# Robot package

This package stores the robot model. 

## Spawn a Robot
To spawn a robot in ```gazebo```, run:
```
  ./scripts/robot_spawn.zsh -r dummy
``` 

You should the following robot:
<div align="center">
  <img src="../../../readme/robot_dummy.gif" width="600">
</div>

## Launch World 
To design the world, I used models from [here](http://data.nvision2.eecs.yorku.ca/3DGEMS/). They are included in gazebo's default model path. To be able to visualize download the models and save them to ```.gazebo/models```. 

To launch the world without a robot, run:
```
  ./scripts/world.zsh -r none
```

You should see:
<p align="center">
    <img src="../../../readme/world.png" width="600"/>
</p>

To launch the world with Gazebo and RVIZ, as well as a robot, run:
```
  ./scripts/world.zsh -r robot_name
```
Where:
* ```robot_name``` is ```dummy```. 

You should see either robot as:
<div align="center">
  <img src="../../../readme/world_dummy.png" width="600">
 
</div>

Rviz should look like this:
<p align="center">
   <img src="../../../readme/rviz.png" width="800" />
</p>

## Troubleshooting

If you can't see the models correcly you might need to download them from [here](http://data.nvision2.eecs.yorku.ca/3DGEMS/) or adjust the paths in the ```.world``` or ```.urdf``` files in the [database package](https://github.com/XLabRD/XLB-hugo-sanchez/tree/REFACTOR_readme-cleanup/catkin_ws/src/database).