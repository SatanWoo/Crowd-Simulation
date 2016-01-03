//
//  Agent.cpp
//  Crowd Simulation
//
//  Created by z on 15-5-14.
//  Copyright (c) 2015年 SatanWoo. All rights reserved.
//

#include "Agent.h"


int Agent::maxForce = 20;
int Agent::maxSpeed = 4;
float32 Agent::radius = 0.23;
float32 Agent::minSeparation = 1.5;
int Agent::maxCohesion = 2;
int Agent::maxForceSquared = Agent::maxForce * Agent::maxForce;
int Agent::maxSpeedSquared = Agent::maxSpeed * Agent::maxSpeed;