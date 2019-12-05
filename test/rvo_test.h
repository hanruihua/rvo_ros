#ifndef NAV_ROV_H
#define NAV_ROV_H

#include "RVO.h"
#include "iostream"

void setupScenario(RVO::RVOSimulator* sim);
void updateVisualization(RVO::RVOSimulator* sim);
bool reachedGoal(RVO::RVOSimulator* sim);
void setPreferredVelocities(RVO::RVOSimulator* sim);

#endif