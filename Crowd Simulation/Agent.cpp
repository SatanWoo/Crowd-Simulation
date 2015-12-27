//
//  Agent.cpp
//  Crowd Simulation
//
//  Created by z on 15-5-14.
//  Copyright (c) 2015年 SatanWoo. All rights reserved.
//

#include "Agent.h"


float32 Agent::MAX_FORCE = 20;
float32 Agent::MAX_SPEED = 4;
float32 Agent::RADIUS = 1;
float32 Agent::MIN_SEPARATION = 0.6;
float32 Agent::MAX_COHESION = 2;
float32 Agent::MAX_FORCE_SQUARED = Agent::MAX_FORCE * Agent::MAX_FORCE;
float32 Agent::MAX_SPEED_SQUARED = Agent::MAX_SPEED * Agent::MAX_SPEED;