#ifndef RVO_GAZEBO_H
#define RVO_GAZEBO_H

#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "gazebo_msgs/ModelStates.h"
#include <string>
#include "../rvo_lib/nav_rvo.h"

int num_agent;  //default
std::vector<std::string> list_obs_name;
float vel_ratio(float vel, float lo, float hi);
// geometry_msgs::Twist *list_obs_twist = new geometry_msgs::Twist();

gazebo_msgs::ModelStates msg_pub;

RVO::RVOPlanner* rvo;


void obs_velCallback(const gazebo_msgs::ModelStates::ConstPtr& sub_msg);















#endif