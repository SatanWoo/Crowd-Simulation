//
//  VirtualNode.h
//  Crowd Simulation
//
//  Created by z on 15/6/2.
//  Copyright (c) 2015年 SatanWoo. All rights reserved.
//

#ifndef Crowd_Simulation_VirtualNode_h
#define Crowd_Simulation_VirtualNode_h

#include <vector>
#include <limits.h>
#include "Box2D.h"
#include "Particle.h"
#include "SimulationController.h"

using namespace std;

struct VirtualNode : public Particle
{
    double maxX, maxY;
    double minX, minY;
    
    double radius;
    double mass;
    int group;
    
    b2Vec2 goal;
    b2Vec2 center;
    b2Vec2 velocity;
    
    vector<int> IDs;
    
    //SimulationController *sc;
    
    VirtualNode(const std::vector<int> IDs)
    {
        this->IDs.assign(IDs.begin(), IDs.end());
        
        mass = 0;
        center = b2Vec2_zero;
        velocity = b2Vec2_zero;
        continuumForce = b2Vec2_zero;
        flockForce = b2Vec2_zero;
    }
    
    virtual ~VirtualNode()
    {
        
    }
    
    void build()
    {
        maxX = numeric_limits<double>::max();
        maxY = numeric_limits<double>::max();
        
        minX = numeric_limits<double>::min();
        minY = numeric_limits<double>::min();
        
        for (size_t i = 0; i < IDs.size(); i++)
        {
//            const RVO::RVOAgent* ai = sc->getAgentsByID(IDs[i]);
//            
//            b2Vec2 pos = ai->getPosition();
//            b2Vec2 vec = ai->getVelocity();
//            center += pos;
//            velocity += vec;
//            
//            if (pos.x > maxX) maxX = pos.x;
//            if (pos.x < minX) minX = pos.x;
//            
//            if (pos.y > maxY) maxY = pos.y;
//            if (pos.y < minY) minY = pos.y;
        }
        
        center *= 1 / this->IDs.size();
        velocity *= 1 / this->IDs.size();
        
        double xDiff = maxX - minX;
        double yDiff = maxY - minY;
        radius = sqrt(xDiff * xDiff + yDiff * yDiff);
    }
    
    void dispatch(double delta)
    {
//        int size = agents.size();
//        for (int i = 0; i < size; i++)
//        {
//            const RVO::RVOAgent *ag = allNodes[i];
//            ai->body->ApplyLinearImpulse(force * delta, ai->getPosition());
//        }
    }
};

#endif
