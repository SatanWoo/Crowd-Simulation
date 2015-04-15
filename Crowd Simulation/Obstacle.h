//
//  Obstacle.h
//  Crowd Simulation
//
//  Created by z on 15-4-15.
//  Copyright (c) 2015年 SatanWoo. All rights reserved.
//

#ifndef Crowd_Simulation_Obstacle_h
#define Crowd_Simulation_Obstacle_h

#include "Terrain.h"

class Obstacle : public Terrain
{
public:
    Obstacle(double x, double y, double z);
    virtual ~Obstacle();
    virtual int obstacleCoefficient()const{return 1999999;}
};


#endif
