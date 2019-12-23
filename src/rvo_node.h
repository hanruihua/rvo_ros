#ifndef RVO_NODE__GAZEBO_H
#define RVO_NODE__GAZEBO_H

#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Point.h"
#include "std_msgs/Header.h"
#include "gazebo_msgs/ModelStates.h"
#include "gazebo_msgs/WorldState.h"
#include <string>
#include "../rvo_lib/nav_rvo.h"
#include "rvo_ros/SetGoals.h"
#include "master_msgs/global_info.h"

const int num_max = 10;
int num_agent = 0;
int copy_num_agent = 1;
float vel_ratio(float vel, float lo, float hi);
// geometry_msgs::Twist *list_obs_twist = new geometry_msgs::Twist();
ros::Publisher rvo_node_pub;
gazebo_msgs::WorldState msg_pub;
std::vector<geometry_msgs::Point> rvo_goals;

RVO::RVOPlanner* rvo;
std::string motion_model = "default";

void rvo_velCallback(const gazebo_msgs::ModelStates::ConstPtr& sub_msg);

bool set_goals(rvo_ros::SetGoals::Request &req, rvo_ros::SetGoals::Response &res);
void rvo_goals_init();
float limit_goal[4];

#endif