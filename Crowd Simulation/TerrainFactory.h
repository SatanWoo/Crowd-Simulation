//
//  TerrainFactory.h
//  Crowd Simulation
//
//  Created by z on 15-4-15.
//  Copyright (c) 2015年 SatanWoo. All rights reserved.
//

#ifndef Crowd_Simulation_TerrainFactory_h
#define Crowd_Simulation_TerrainFactory_h


#include "Obstacle.h"

typedef enum
{
    TerrainNormal = 0,
    TerrainObstacle = 1
} TerrainType;


class TerrainFactory
{
public:
    static Terrain *createTerrain(TerrainType type, double x, double y, double z)
    {
        if (type == TerrainNormal)
        {
            return new Terrain(x, y, z);
        }
        else
        {
            return new Obstacle(x, y, z);
        }
    }
};

#endif
