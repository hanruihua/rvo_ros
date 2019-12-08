#include "rvo_node.h"

int main(int argc, char **argv)
{
   
    ros::init(argc, argv, "rvo_node");
    ros::NodeHandle n;
    ros::Publisher rvo_node_pub = n.advertise<gazebo_msgs::ModelStates>("rvo_vel",1000);
    ros::Subscriber sub = n.subscribe("/rvo/model_states", 1000, rvo_velCallback);
    ros::Rate loop_rate(10);
 
    rvo = new RVO::RVOPlanner("gazebo");
    rvo->setupScenario(4.0f, 10, 10.0f, 5.0f, 0.25f, 0.3f);

    while (ros::ok())
    {
        rvo_node_pub.publish(msg_pub);
        ros::spinOnce();
        loop_rate.sleep();
    }
}

void rvo_velCallback(const gazebo_msgs::ModelStates::ConstPtr& sub_msg)
{
    // std::cout<<num_agent<<std::endl;
    rvo->updateState_gazebo(sub_msg); // read the message
    rvo->randGoal(0, 7, 0, 4);   // set the goals
    rvo->setInitial();
    rvo->setPreferredVelocities();

    std::vector<RVO::Vector2*> new_velocities = rvo->step();
    
    auto models_name = sub_msg->name;

    msg_pub.name.clear();
    msg_pub.twist.clear();
    msg_pub.pose.clear();

    for (int i = 0; i < new_velocities.size(); i++)
    {
        geometry_msgs::Twist new_vel;
        geometry_msgs::Pose rvo_pose;
        std::string agent_name = "angent" + std::to_string(i);

        auto iter_agent = std::find(models_name.begin(), models_name.end(), agent_name);
        int agent_index = iter_agent - models_name.begin();

        rvo_pose = sub_msg->pose[agent_index];

        float x = new_velocities[i]->x();
        float y = new_velocities[i]->y();

        // float speed = sqrt(x * x + y * y);

        // float ratio = vel_ratio(speed, 0.2f, 0.3f);

        new_vel.linear.x = x;
        new_vel.linear.y = y;

        msg_pub.name.push_back(agent_name);
        msg_pub.twist.push_back(new_vel);
        msg_pub.pose.push_back(rvo_pose);
    }
}

float vel_ratio(float vel, float lo, float hi)
{
    return (vel < lo) ? lo/vel : (vel > hi) ? hi/vel : 1;
}