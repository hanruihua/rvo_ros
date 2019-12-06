#include "rvo_test.h"

std::vector <RVO::Vector2> goals;

int main()
{
  // Create a new simulator instance.
  RVO::RVOSimulator* sim = new RVO::RVOSimulator();

  // Set up the scenario.
  setupScenario(sim);

  // Perform (and manipulate) the simulation.
  do {
    updateVisualization(sim);
    setPreferredVelocities(sim);
    sim->doStep();
  } while (!reachedGoal(sim));

  delete sim;
}

void setupScenario(RVO::RVOSimulator* sim){
    sim->setTimeStep(0.25f);

    sim->setAgentDefaults(15.0f, 10, 10.0f, 5.0f, 2.0f, 2.0f);

    sim->addAgent(RVO::Vector2(-50.0f, -50.0f));
    sim->addAgent(RVO::Vector2(50.0f, -50.0f));
    sim->addAgent(RVO::Vector2(50.0f, 50.0f));
    sim->addAgent(RVO::Vector2(-50.0f, 50.0f));

    for (size_t i=0; i < sim->getNumAgents(); ++i){
        goals.push_back(-sim->getAgentPosition(i));
    }

    std::vector<RVO::Vector2> vertices;

    vertices.push_back(RVO::Vector2(-7.0f, -20.0f));
    vertices.push_back(RVO::Vector2(7.0f, -20.0f));
    vertices.push_back(RVO::Vector2(7.0f, 20.0f));
    vertices.push_back(RVO::Vector2(-7.0f, 20.0f));

    sim->addObstacle(vertices);

    sim->processObstacles();
}

void updateVisualization(RVO::RVOSimulator* sim){

    std::cout<< sim->getGlobalTime()<<"";

    for (size_t i=0; i<sim->getNumAgents(); ++i){
        std::cout<<sim->getAgentPosition(i)<<"";
    }
 
    std::cout<<std::endl;
}
bool reachedGoal(RVO::RVOSimulator* sim) {
  // Check whether all agents have arrived at their goals.
  for (size_t i = 0; i < sim->getNumAgents(); ++i) {
    if (absSq(goals[i] - sim->getAgentPosition(i)) > sim->getAgentRadius(i) * sim->getAgentRadius(i)) {
      // Agent is further away from its goal than one radius.
      return false;
    }
  }
  return true;
}

void setPreferredVelocities(RVO::RVOSimulator* sim) {
  // Set the preferred velocity for each agent.
  for (size_t i = 0; i < sim->getNumAgents(); ++i) {
    if (absSq(goals[i] - sim->getAgentPosition(i)) < sim->getAgentRadius(i) * sim->getAgentRadius(i) ) {
      // Agent is within one radius of its goal, set preferred velocity to zero
      sim->setAgentPrefVelocity(i, RVO::Vector2(0.0f, 0.0f));
    } else {
      // Agent is far away from its goal, set preferred velocity as unit vector towards agent's goal.
      sim->setAgentPrefVelocity(i, normalize(goals[i] - sim->getAgentPosition(i)));
    }
  }
}

