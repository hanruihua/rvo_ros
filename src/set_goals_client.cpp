#include "ros/ros.h"
#include "geometry_msgs/Point.h"
#include "rvo_ros/SetGoals.h"


int main(int argc, char **argv)
{
    ros::init(argc, argv, "set_goals_client");
    ros::NodeHandle n;
    ros::ServiceClient client = n.serviceClient<rvo_ros::SetGoals>("set_rvo_goals");

    rvo_ros::SetGoals srv;

    srv.request.model = argv[1];
    int args_num = argc - 1;
    int coordinate_num = argc - 2;

    std::cout<<"arguments input:"<<std::endl;

    if (args_num == 0)
        ROS_ERROR("Please input the args");

    if (coordinate_num % 2 != 0)
    {
        std::cout << "The num of coordinates is wrong" << std::endl;
        return 1;
    }

    if (srv.request.model == "default")
    {
        for (int i = 2; i < coordinate_num + 2; i = i + 2)
        {
            geometry_msgs::Point point;
            point.x = atof(argv[i]);
            point.y = atof(argv[i + 1]);
            srv.request.coordinates.push_back(point);
        }
    }

    else if (srv.request.model == "random")
    {
        if (args_num < 5)
        {
            std::cout << "The num of coordinates is wrong" << std::endl;
            return 1;
        }

        geometry_msgs::Point point1;
        geometry_msgs::Point point2;

        point1.x = atof(argv[2]);
        point2.x = atof(argv[3]);
        point1.y = atof(argv[4]);
        point2.y = atof(argv[5]);

        srv.request.coordinates.push_back(point1);
        srv.request.coordinates.push_back(point2);

        std::cout<<"Request has been sent"<<std::endl;
        std::cout<<"model: random"<<std::endl;
        std::cout<<"limit x"<<"["<<point1.x<<","<<point2.x<<"]"<<std::endl;
        std::cout<<"limit y"<<"["<<point1.y<<","<<point2.y<<"]"<<std::endl;
    }

    else if (srv.request.model == "circle")
    {
        if (args_num < 3)
        {
            std::cout << "The num of coordinates is wrong" << std::endl;
            return 1;
        }
        
        geometry_msgs::Point point1;
        geometry_msgs::Point point2;

        // circle point
        point1.x = atof(argv[2]);
        point1.y = atof(argv[3]);

        // radius
        point2.x = atof(argv[4]);
        point2.y = atof(argv[5]);

        srv.request.coordinates.push_back(point1);
        srv.request.coordinates.push_back(point2);
        
    }

    else if (srv.request.model == "circle_spin")
    {
        if (args_num < 2)
        {
            std::cout << "The num of coordinates is wrong" << std::endl;
            return 1;
        }
        
        geometry_msgs::Point point1;
        geometry_msgs::Point point2;

        // circle point
        point1.x = atof(argv[2]);
        point1.y = atof(argv[3]);

        // radius
        point2.x = atof(argv[4]);
        point2.y = atof(argv[4]);

        srv.request.coordinates.push_back(point1);
        srv.request.coordinates.push_back(point2);
        
    }


    if (client.call(srv))
    {
        ROS_INFO("call service successfully, number of goals: %ld", srv.response.num_goal);
    }

    else
    {
        ROS_ERROR("call service fail");
        return 1;
    }

    return 0;
    
}
