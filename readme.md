# Optimal Reciprocal Collision Avoidance 

[[paper]](http://citeseerx.ist.psu.edu/viewdoc/download?doi=10.1.1.162.265&rep=rep1&type=pdf)

## Introduction

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

## Step

### Initialization

1. environment:
    - time horizon(float):  
2. robot:
    - position (vector): current position in 2d environment.
    - current velocity(vector, x,y): current velocity respect to x and y axis.

### set scenario

1. set time step
2. set default parameter of robots: 
    - preferred velocity
    - 
3. set robot initial position
4. 




