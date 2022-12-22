# reciprocal velocity obstacle ros library

## Introduction

This ros package is derived from the ORCA library ([lib](http://gamma.cs.unc.edu/RVO2/)).

![](https://github.com/hanruihua/rvo_ros/blob/master/simulation/rvo_sim.gif)

## Environment

- Ubuntu 18.04
- ros Melodic

## Install

> git clone https://github.com/hanruihua/rvo_ros.git  
> cd ~/catkin_ws  
> catkin_make  

## set environment parameter

> export GAZEBO_PLUGIN_PATH=${GAZEBO_PLUGIN_PATH}:~/catkin_ws/devel/lib

Please write this line in the file .bashrc or .zshrc

## Usage

> rosrun rvo_ros rvo_node args

args: the coordinates of init point. default 0,1 0,2 ...0 10

for example: 

> rosrun rvo_ros rvo_node 0 1 0 2 0 3 

## test with simulation

> roslaunch rvo_ros rvo_gazebo_agent.launch

**Note**: Using service to set the model and goals. 

## Service

> rosrun rvo_ros set_goals_client

- arguments:
    - model:
        - "default": specify a series of point as goals for the agents. The number of goals should be same as the number of agents: 1 1 2 3 4 2.
        - "random": allocate the goals randomly with limit along x and y, only for number: min_x, max_x, min_y, max_y.
        - "circle": allocate the goals with circle shape: circle_point_x, circle_point_y, radius, flag. flag is to set the reverse mode

- example:
    >rosrun rvo_ros set_goals_client default 1 1 1 4 4 4 4 1  
    >rosrun rvo_ros set_goals_client random 0 5 1 4   
    >rosrun rvo_ros set_goals_client circle 4 4 4 0 

## Topics

- Subscribed Topic

/rvo/model_states ([gazebo_msgs/ModelStates](http://docs.ros.org/jade/api/gazebo_msgs/html/msg/ModelStates.html))

**attention**: To avoid the model confusion, only the model name which is like the 'agent+num' style, for example, agent1, agent2, can be regarded as the agent model.

- Published Topic

/rvo_vel ([gazebo_msgs/WorldStates](http://docs.ros.org/jade/api/gazebo_msgs/html/msg/WorldState.html))

**Note**: only the speed in x, y direction of each agent calculated from the rvo are set in the WorldStates twist part. 


## author

**Han** - [Han](https://github.com/hanruihua)  

## License

This project is licensed under the MIT License

## Overview ([[paper]](http://citeseerx.ist.psu.edu/viewdoc/download?doi=10.1.1.162.265&rep=rep1&type=pdf))

The approach for reciprocal n-body collision avoidance, where multiple mobile robots need to avoid collisions with each other while moving in a common workspace.

Assumption:

1. Each robot is assumed to have a simple shape (circular or convex polygon) moving in a two-dimensional workspace.
2. The robot is holonomic, i.e. it can move in any direction, such that the control input of each robot is simply given by a two-dimensional velocity vector.
3. Each robot has perfect sensing, and is able to infer the exact shape, position and velocity of obstacles and other robots in the environment.

Advantage:

1. Do not need communication among robots.
2. Can tackle the static obstacles.
3. Can guarantee local collision-free motion for a large number of robots in a cluttered workspace.

Limitation:

1. The assumption of perfect sensing is hard to perform in real world because of the uncertainties.
2. Too many parameters to construct the complex model.  






