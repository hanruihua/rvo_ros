#!/bin/bash
rosrun rvo_ros set_goals_client default 4 3 5 3 6 3 6 4 6 5 5 5 4 5 4 6 4 7 5 7 6 7    # S

sleep 10

rosrun rvo_ros set_goals_client default 4 3 5 3 6 3 6 4 6 5 4 4 4 5 4 6 4 7 6 6 6 7    # U

sleep 4

rosrun rvo_ros set_goals_client default 4 3 5 3 6 3 6 4 6 5 5 5 4 5 4 6 4 7 5 7 6 7    # S

sleep 4

rosrun rvo_ros set_goals_client default 5 2 5 3 5 4 5 5 5 6 5 7 5 8 3 8 4 8 6 8 7 8   # T

sleep 6

rosrun rvo_ros set_goals_client default 4 3 5 3 6 3 4 6 6 5 5 5 4 5 4 4 4 7 5 7 6 7   # E

sleep 8

rosrun rvo_ros set_goals_client default 4 3 5 3 6 3 4 6 7 3 7 7 4 5 4 4 4 7 5 7 6 7   # C

sleep 6

rosrun rvo_ros set_goals_client default 4 3 5 5 6 3 4 6 6 4 6 5 4 5 4 4 4 7 6 6 6 7   # H

