# rvo_ros package

## Introduction

This ros package is derived from the ORCA library ([lib](http://gamma.cs.unc.edu/RVO2/)).

## Environment

- Ubuntu 18.0
- ros Melodic

## Install

> git clone https://github.com/hanruihua/rvo_ros.git  
> cd ~/catkin_ws  
> catkin_make  

## Usage

> rosrun rvo_ros rosrun rvo_ros rvo_node_gazebo  

## Topics

- Subscribed Topic

/rvo/model_states ([gazebo_msgs/ModelStates](http://docs.ros.org/jade/api/gazebo_msgs/html/msg/ModelStates.html))

- Published Topic

/rvo_vel ([gazebo_msgs/ModelStates](http://docs.ros.org/jade/api/gazebo_msgs/html/msg/ModelStates.html))

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






