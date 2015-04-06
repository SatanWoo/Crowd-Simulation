//
//  Path.cpp
//  Crowd Simulation
//
//  Created by z on 15-4-6.
//  Copyright (c) 2015年 SatanWoo. All rights reserved.
//

#include <stdio.h>
#include "Path.h"
#include "MapController.h"
#include "Vector.h"
#include "Terrain.h"

Path::Path()
{
}

Path::~Path()
{
    paths.clear();
    m_map = NULL;
}

void Path::render(getTerrain g)
{
    size_t size = paths.size();
    
    for (int i = 0; i < size; i++) {
        Vector2D &vec = paths[i];
        Terrain *t = (m_map->*g)(vec.getX(), vec.getY());
        t->render();
    }
}

void Path::addPath(double x, double y)
{
    paths.push_back(Vector2D(x, y));
}
