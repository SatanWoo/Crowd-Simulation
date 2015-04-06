//
//  Path.h
//  Crowd Simulation
//
//  Created by z on 15-4-6.
//  Copyright (c) 2015年 SatanWoo. All rights reserved.
//

#ifndef Crowd_Simulation_Path_h
#define Crowd_Simulation_Path_h
#include <vector>

class Terrain;
class MapController;

typedef Terrain* (MapController::*getTerrain) (double x, double y);

class Vector2D;
class Path
{
public:
    Path();
    ~Path();
    void setMap(MapController *map) {m_map = map;}
    void addPath(double x, double y);
    void render(getTerrain g);
protected:
    std::vector<Vector2D> paths;
    MapController *m_map;
};

#endif
