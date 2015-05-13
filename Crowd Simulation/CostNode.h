//
//  CostNode.h
//  Crowd Simulation
//
//  Created by z on 15-5-7.
//  Copyright (c) 2015年 SatanWoo. All rights reserved.
//

#ifndef Crowd_Simulation_CostNode_h
#define Crowd_Simulation_CostNode_h

#include "Box2D.h"

struct CostNode {
    
    float32 cost;
    b2Vec2 point;

    bool operator< (const CostNode& v) const {
        return cost > v.cost;
    }
};

struct FourGrid {
    float32 value[4];
};


#endif