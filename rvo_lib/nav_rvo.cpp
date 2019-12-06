#include "nav_rvo.h"

namespace RVO
{

RVOPlanner::RVOPlanner(std::string simulation) : simulator(simulation)
{
    sim = new RVO::RVOSimulator();
};

void RVOPlanner::setupScenario(float neighborDist, size_t maxNeighbors, float timeHorizon, float timeHorizonObst, float radius, float maxSpeed)
{
    sim->setAgentDefaults(neighborDist, maxNeighbors, timeHorizon, timeHorizonObst, radius, maxSpeed);
}

// set the goal manually
// default: invert direction
// random: change the goal every step

void RVOPlanner::setGoal()
{
    for (size_t i = 0; i < sim->getNumAgents(); ++i)
    {
        if (absSq(goals[i] - sim->getAgentPosition(i)) < goal_threshold)
        {
            // goals[i] = -sim->getAgentPosition(i);
            goals[i] = -goals[i];
        }
    }
}

void RVOPlanner::setGoal(std::vector<RVO::Vector2> set_goals)
{
    goals.clear();
    goals.assign(set_goals.begin(), set_goals.end());
}

void RVOPlanner::randGoal(int x_min, int x_max, int y_min, int y_max, std::string model)
{
    std::srand((unsigned)time(NULL));

    for (size_t i = 0; i < sim->getNumAgents(); ++i)
    {

        float x = x_min + std::rand() % (x_max - x_min + 1);
        float y = y_min + std::rand() % (y_max - y_min + 1);
        int rand = std::rand() % 10;
        // float x = (rand() % (limit_x + 1));
        // float y = (rand() % (limit_y + 1));
        if (IfInitial == false)
            goals.push_back(Vector2(x, y));
        else if (model == "default")
        {
            if (absSq(goals[i] - sim->getAgentPosition(i)) < goal_threshold)
                goals[i] = (Vector2(x, y));
        }
        else if (model == "random")
        {
            if (rand > 2)
                goals[i] = (Vector2(x, y));       
        }
    }
}

void RVOPlanner::setInitial()
{
    if ((goals.size() != 0) && (sim->agents_.size() != 0))
        IfInitial = true;
    else
        IfInitial = false;
}

void RVOPlanner::setPreferredVelocities()
{
    for (size_t i = 0; i < sim->getNumAgents(); ++i)
    {
        if (absSq(goals[i] - sim->getAgentPosition(i)) < goal_threshold)
        {
            // Agent is within one radius of its goal, set preferred velocity to zero
            sim->setAgentPrefVelocity(i, RVO::Vector2(0.0f, 0.0f));
        }
        else
        {
            sim->setAgentPrefVelocity(i, normalize(goals[i] - sim->getAgentPosition(i)));
        }
    }
}

void RVOPlanner::updateState_gazebo(gazebo_msgs::ModelStates::ConstPtr model_msg)
{
    if (simulator == "gazebo")
    {
        auto models_name = model_msg->name;
        int num = models_name.size();

        for (int i = 0; i < num; i++)
        {
            std::string agent_name = "agent" + std::to_string(i);

            auto iter_agent = std::find(models_name.begin(), models_name.end(), agent_name);
            int agent_index = iter_agent - models_name.begin();
            if (iter_agent != models_name.end())
            {
                float obs_x = model_msg->pose[agent_index].position.x;
                float obs_y = model_msg->pose[agent_index].position.y;

                if (IfInitial == true)
                    sim->agents_[i]->position_ = RVO::Vector2(obs_x, obs_y);
                else
                    sim->addAgent(RVO::Vector2(obs_x, obs_y));
            }
        }
    }
    else
        std::cout << "error: please check the simulator" << std::endl;
}

std::vector<RVO::Vector2 *> RVOPlanner::step()
{
    sim->kdTree_->buildAgentTree();
    newVelocities.clear();

    for (int i = 0; i < static_cast<int>(sim->agents_.size()); ++i)
    {
        sim->agents_[i]->computeNeighbors();
        sim->agents_[i]->computeNewVelocity();
        RVO::Vector2 *new_vel = new Vector2(sim->agents_[i]->newVelocity_.x(), sim->agents_[i]->newVelocity_.y());
        newVelocities.push_back(new_vel);
    }

    return newVelocities;
}

}; // namespace RVO