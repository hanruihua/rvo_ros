#include "rvo_gazebo.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "obs_vel");
    ros::NodeHandle n;
    ros::Publisher obs_vel_pub = n.advertise<gazebo_msgs::ModelStates>("obstacles_velocity",1000);
    ros::Subscriber sub = n.subscribe("/gazebo/model_states", 1000, obs_velCallback);
    ros::Rate loop_rate(10);
 
    if (argc == 2)
        num_agent = atoi(argv[1]);
    else
        num_agent = 1;
        
    for (int i=0; i<num_agent; i++)
    {
        list_obs_name.push_back("obstacle" + std::to_string(i));
    }

    rvo = new RVO::RVOPlanner("gazebo", num_agent);

    rvo->setupScenario(4.0f, 10, 10.0f, 5.0f, 0.25f, 0.3f);

    while (ros::ok())
    {
        obs_vel_pub.publish(msg_pub);
        ros::spinOnce();
        loop_rate.sleep();
    }
}

void obs_velCallback(const gazebo_msgs::ModelStates::ConstPtr& sub_msg)
{
    // std::cout<<num_agent<<std::endl;

    rvo->updateState_gazebo(sub_msg);
    rvo->randGoal(0, 9, 0, 4);
    rvo->setInitial();
    rvo->setPreferredVelocities();

    std::vector<RVO::Vector2*> new_velocities = rvo->step();

    if (new_velocities.size() == num_agent)
    {
        msg_pub.name.clear();
        msg_pub.twist.clear();

        for (int i = 0; i < num_agent; i++)
        {
            geometry_msgs::Twist new_vel;
            
            float x = new_velocities[i]->x();
            float y = new_velocities[i]->y();

            float speed = sqrt(x * x + y * y);

            float ratio = vel_ratio(speed, 0.2f, 0.3f);

            new_vel.linear.x = x * ratio;
            new_vel.linear.y = y * ratio;

            msg_pub.name.push_back(list_obs_name[i]);
            msg_pub.twist.push_back(new_vel);
        }
    }
    else
        std::cout<<"The number of the anent is wrong"<<std::endl;
}

float vel_ratio(float vel, float lo, float hi)
{
    return (vel < lo) ? lo/vel : (vel > hi) ? hi/vel : 1;
}