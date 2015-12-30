//
//  VirtualNode.h
//  Crowd Simulation
//
//  Created by z on 15/6/2.
//  Copyright (c) 2015年 SatanWoo. All rights reserved.
//

#ifndef Crowd_Simulation_VirtualNode_h
#define Crowd_Simulation_VirtualNode_h

#include <iostream>
#include <vector>
#include "Box2D.h"
#include "Agent.h"

using namespace std;

class VirtualNode : public Agent
{
public:
    double maxX;
    double maxY;
    
    double minX;
    double minY;
    
    b2Vec2 impulse;
    
    std::vector<Agent *> allNodes;
    
    Agent *leader_;
    
    static float32 Leading_Weight;
    
    // :Agent(leader->getPosition(), leader->group)

    
    VirtualNode(Agent *leader, std::vector<Agent *> &neighbours)
    {
        leader_ = leader;
        goal = leader_->goal;
        
//        allNodes.assign(neighbours.begin(), neighbours.end());
        
//        maxX = INT_MIN;
//        maxY = INT_MIN;
//        
//        minX = INT_MAX;
//        minY = INT_MAX;
        
//        impulse = b2Vec2_zero;
//        b2Vec2 center = b2Vec2_zero;
//        
//        if (leader->pos.x > maxX) maxX = leader->pos.x;
//        if (leader->pos.x < minX) minX = leader->pos.x;
//        
//        if (leader->pos.y > maxY) maxY = leader->pos.y;
//        if (leader->pos.y < minY) minY = leader->pos.y;
//        
//        size_t size = allNodes.size();
//        for (size_t i = 0; i < size; ++i)
//        {
//            Agent *ai = allNodes[i];
//            
//            b2Vec2 pos = ai->getPosition();
//            center += pos;
//            
//            if (pos.x > maxX) maxX = pos.x;
//            if (pos.x < minX) minX = pos.x;
//            
//            if (pos.y > maxY) maxY = pos.y;
//            if (pos.y < minY) minY = pos.y;
//        }
        
//        size += 1;
//        center += VirtualNode::Leading_Weight * leader->getPosition();
//        center *= 1 / size;
//        
        group = leader->group;
        pos = leader->getPosition();
        
//        double xDiff = maxX - minX;
//        double yDiff = maxY - minY;
//        radius_ = sqrt(xDiff * xDiff + yDiff * yDiff);
//        
//        if (size == 1) {
//            radius_ = Agent::RADIUS;
//        }
    }
    
    void dispatch(double delta)
    {
//        int size = allNodes.size();
//        for (int i = 0; i < size; i++)
//        {
//            Agent *ai = allNodes[i];
//            ai->body->ApplyLinearImpulse(impulse, ai->getPosition());
//        }
//        
//        //std::cout << leader_->getPosition().x << ":" << leader_->getPosition().y << std::endl;
//        leader_->body->ApplyLinearImpulse(impulse, leader_->getPosition());
    }
};

#endif
