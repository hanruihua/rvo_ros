#include "rvo_node.h"

int main(int argc, char **argv)
{

    ros::init(argc, argv, "rvo_node");
    ros::NodeHandle n;
    rvo_node_pub = n.advertise<gazebo_msgs::ModelStates>("rvo_vel", 1000);
    ros::Subscriber sub = n.subscribe("/rvo/model_states", 1000, rvo_velCallback);
    ros::ServiceServer service = n.advertiseService("set_rvo_goals", set_goals);
    ros::Rate loop_rate(10);

    if ((argc > 1) && (argc % 2 == 1))
    {
        int num_init_point = argc - 1;
        for (int i = 1; i < num_init_point + 1; i = i + 2)
        {
            geometry_msgs::Point point;
            point.x = atof(argv[i]);
            point.y = atof(argv[i + 1]);
            rvo_goals.push_back(point);
        }
    }

    rvo = new RVO::RVOPlanner("gazebo");
    rvo->setupScenario(4.0f, 10, 18.0f, 5.0f, 0.25f, 0.3f);
    rvo_goals_init();
    while (ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }
}

bool set_goals(rvo_ros::SetGoals::Request &req, rvo_ros::SetGoals::Response &res)
{
    if (req.model == "default")
    {
        motion_model = req.model;

        if (!rvo_goals.empty())
            rvo_goals.clear();

        for (const auto &coordinate : req.coordinates)
        {
            rvo_goals.push_back(coordinate);
        }

        res.num_goal = rvo_goals.size();

        return true;
    }

    if (req.model == "random")
    {
        motion_model = req.model;
        if (req.coordinates.size() < 2)
        {
            ROS_ERROR("too less input");
            return false;
        }
        else
        {
            limit_goal[0] = req.coordinates[0].x; // x_min
            limit_goal[1] = req.coordinates[1].x; // x_max
            limit_goal[2] = req.coordinates[0].y; // y_min
            limit_goal[3] = req.coordinates[1].y; // y_max

            res.num_goal = num_agent;
            std::cout << "Current number of agent: " << num_agent << std::endl;
            return true;
        }
    }

    std::cout << "The specific model is wrong" << std::endl;
    return false;
}

void rvo_goals_init()
{

    if (rvo_goals.empty())
    {
        for (int i = 0; i < num_max; i++)
        {
            geometry_msgs::Point point;
            point.x = float(i);
            point.y = 1.0;
            rvo_goals.push_back(point);
        }
    }
}

void rvo_velCallback(const gazebo_msgs::ModelStates::ConstPtr &sub_msg)
{
    // std::cout<<num_agent<<std::endl;
    rvo->updateState_gazebo(sub_msg); // read the message
    if (motion_model == "default")
        rvo->setGoal(rvo_goals);
    else if (motion_model == "random")
        rvo->randGoal(limit_goal, "default");

    rvo->setInitial();
    rvo->setPreferredVelocities();

    std::vector<RVO::Vector2 *> new_velocities = rvo->step();

    auto models_name = sub_msg->name;
    int total_num = models_name.size();

    msg_pub.name.clear();
    msg_pub.twist.clear();
    msg_pub.pose.clear();

    num_agent = new_velocities.size();

    if (num_agent != copy_num_agent)
    {
        std::cout << "The num of agents is" + std::to_string(num_agent) << std::endl;
        copy_num_agent = num_agent;
    }

    int count = 0;

    for (int i = 0; i < total_num; i++)
    {
        geometry_msgs::Twist new_vel;
        geometry_msgs::Pose rvo_pose;
        std::string agent_name = "agent" + std::to_string(i + 1);

        auto iter_agent = std::find(models_name.begin(), models_name.end(), agent_name);
        int agent_index = iter_agent - models_name.begin();

        if (iter_agent != models_name.end())
        {
            rvo_pose = sub_msg->pose[agent_index];

            float x = new_velocities[count]->x();
            float y = new_velocities[count]->y();

            // float speed = sqrt(x * x + y * y);

            // float ratio = vel_ratio(speed, 0.2f, 0.3f);

            new_vel.linear.x = x;
            new_vel.linear.y = y;

            msg_pub.name.push_back(agent_name);
            msg_pub.twist.push_back(new_vel);
            msg_pub.pose.push_back(rvo_pose);

            count++;
            std::cout << "Current " << agent_name << std::endl;
        }
    }
    rvo_node_pub.publish(msg_pub);
}

float vel_ratio(float vel, float lo, float hi)
{
    return (vel < lo) ? lo / vel : (vel > hi) ? hi / vel : 1;
}