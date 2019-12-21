#ifndef NAV_ROV_H_
#define NAV_ROV_H_

#include "Agent.h"
#include "KdTree.h"
#include "Definitions.h"
#include "Obstacle.h"
#include "gazebo_msgs/ModelStates.h"
#include "geometry_msgs/Point.h"
#include "RVOSimulator.h"
#include <string>
#include <random>

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
        void randGoal(const float limit_goal[4], const std::string &model="default");
        void randomOnceGoal(const float limit_goal[4]);
        void setGoal(std::vector<geometry_msgs::Point> set_goals);
        void setInitial();
        void setPreferredVelocities();

        std::vector<RVO::Vector2*>  step();

    private:

        RVO::RVOSimulator* sim;
        std::string simulator;
        std::vector <RVO::Vector2> goals;
        bool IfInitial = false;
        std::vector<RVO::Vector2 *> newVelocities;
        float goal_threshold = 0.04;

        friend class Agent;
        friend class KdTree;
        friend class Obstacle;
    };
}

#endif