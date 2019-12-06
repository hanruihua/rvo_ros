#ifndef NAV_ROV_H
#define NAV_ROV_H

#include "Agent.h"
#include "KdTree.h"
#include "Definitions.h"
#include "Obstacle.h"
#include "gazebo_msgs/ModelStates.h"
#include "RVOSimulator.h"
#include <string>

namespace RVO {

    class Agent;
    class Obstacle;
    class KdTree;

    class RVOPlanner{
    public:
        RVOPlanner(std::string simulation);

        void setupScenario(float neighborDist, size_t maxNeighbors, float timeHorizon, float timeHorizonObst, float radius, float maxSpeed);

        void updateState_gazebo(gazebo_msgs::ModelStates::ConstPtr model_msg);

        void setGoal();
        void randGoal(int x_min, int x_max, int y_min, int y_max, std::string model="default");
        void setGoal(std::vector <RVO::Vector2> set_goals);
        void setInitial();
        void setPreferredVelocities();

        std::vector<RVO::Vector2*>  step();

        ~RVOPlanner();

    private:

        RVO::RVOSimulator* sim;
        std::string simulator;
        std::vector <RVO::Vector2> goals;
        bool IfInitial = false;
        std::vector<RVO::Vector2 *> newVelocities;
        float goal_threshold = 0.01;
        int limit_goal_x;
        int limit_goal_y;

        friend class Agent;
        friend class KdTree;
        friend class Obstacle;
    };
};















#endif