#include "nav_rvo.h"

#include <utility>

namespace RVO
{

RVOPlanner::RVOPlanner(std::string simulation) : simulator(std::move(simulation))
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

void RVOPlanner::setGoal(std::vector<geometry_msgs::Point> set_goals)
{
    goals.clear();
    int num_agent = sim->agents_.size();
    if (set_goals.size() < num_agent)
        std::cout << "error:The num of goals is less than agents" << std::endl;
    else
    {
        for (int i = 0; i < num_agent; i++)
        {
            float x = set_goals[i].x;
            float y = set_goals[i].y;

            goals.emplace_back(Vector2(x, y));
            std::cout<<"goal"+ std::to_string(i+1) + ":["<<x<<","<<y<<"]"<<std::endl;
        }
    }
}


void RVOPlanner::randomOnceGoal(const float limit_goal[4])
{
    float x_min = limit_goal[0];
    float x_max = limit_goal[1];
    float y_min = limit_goal[2];
    float y_max = limit_goal[3];

    std::random_device rd;
    std::default_random_engine e(rd());
    std::uniform_real_distribution<float> ux(x_min, x_max);
    std::uniform_real_distribution<float> uy(y_min, y_max);
    std::uniform_int_distribution<int> ur(0, 10);

    for (size_t i = 0; i < sim->getNumAgents(); ++i)
    {

        float x = ux(e);
        float y = uy(e);
        int rand = ur(e);
    
        goals[i] = (Vector2(x, y));

        std::cout<< "random once successfully" <<std::endl;
    }
       
}

void RVOPlanner::randGoal(const float limit_goal[4], const std::string &model)
{

    float x_min = limit_goal[0];
    float x_max = limit_goal[1];
    float y_min = limit_goal[2];
    float y_max = limit_goal[3];

    std::random_device rd;
    std::default_random_engine e(rd());
    std::uniform_real_distribution<float> ux(x_min, x_max);
    std::uniform_real_distribution<float> uy(y_min, y_max);
    std::uniform_int_distribution<int> ur(0, 10);

    for (size_t i = 0; i < sim->getNumAgents(); ++i)
    {

        float x = ux(e);
        float y = uy(e);
        int rand = ur(e);

        if (!IfInitial)
            goals.emplace_back(x, y);

        else if (model == "default")
        {
            if (absSq(goals[i] - sim->getAgentPosition(i)) < goal_threshold)
                goals[i] = (Vector2(x, y));
        }
        else if (model == "random")
        {
            if (rand > 8)
                goals[i] = (Vector2(x, y));
        }
    }
}

bool RVOPlanner::arrived()
{
    bool reach = true;

    for (size_t i = 0; i < sim->getNumAgents(); ++i)
    {
        if (absSq(goals[i] - sim->getAgentPosition(i)) >= goal_threshold)
            reach = false;
    }

    return reach;
}



void RVOPlanner::setInitial()
{
    IfInitial = (!goals.empty()) && (!sim->agents_.empty());
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

void RVOPlanner::updateState_gazebo(gazebo_msgs::ModelStates::ConstPtr model_msg, std::string agent_name)
{
    if (simulator == "gazebo")
    {
        auto models_name = model_msg->name;
        int num = models_name.size();
        int count = 0;

        // sim->agents_.clear();

        for (int i = 0; i < num; i++)
        {
            // std::string full_agent_name = agent_name + std::to_string(i+1);
            std::string full_agent_name = agent_name + std::to_string(i);

            auto iter_agent = std::find(models_name.begin(), models_name.end(), full_agent_name);
            int agent_index = iter_agent - models_name.begin();

            if (iter_agent != models_name.end())
            {
                float obs_x = model_msg->pose[agent_index].position.x;
                float obs_y = model_msg->pose[agent_index].position.y;
                float vel_x = model_msg->twist[agent_index].linear.x;
                float vel_y = model_msg->twist[agent_index].linear.y;

               if (IfInitial)
               {
                   sim->agents_[count]->position_ = RVO::Vector2(obs_x, obs_y);
                   sim->agents_[count]->velocity_ = RVO::Vector2(vel_x, vel_y);
               }
                   
               else
                   sim->addAgent(RVO::Vector2(obs_x, obs_y));

                count++;
                // sim->agents_[i]->quater = model_msg->pose[agent_index].orientation;
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

    for (auto & agent : sim->agents_)
    {
        agent->computeNeighbors();
        agent->computeNewVelocity();
        auto *new_vel = new Vector2(agent->newVelocity_.x(), agent->newVelocity_.y());
        newVelocities.push_back(new_vel);
    }

    return newVelocities;
}

}; // namespace RVO