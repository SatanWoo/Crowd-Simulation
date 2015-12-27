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
#include "Box2D.h"
#include "Agent.h"

struct VirtualNode
{
    double maxX;
    double maxY;
    
    double minX;
    double minY;
    
    b2Vec2 center;
    b2Vec2 velocity;
    
    size_t ID_;
    size_t groupID_;
    
    float32 radius_;
    
    std::vector<Agent *> allNodes;
    
    void build()
    {
        maxX = INT_MIN;
        maxY = INT_MIN;
        
        minX = INT_MAX;
        minY = INT_MAX;
        
        center = b2Vec2_zero;
        velocity = b2Vec2_zero;
        
        size_t size = allNodes.size();
        for (size_t i = 0; i < size; ++i)
        {
            Agent *ai = allNodes[i];
            
            b2Vec2 pos = ai->getPosition();
            b2Vec2 vec = ai->getVelocity();
            center += pos;
            velocity += vec;
            
            if (pos.x > maxX) maxX = pos.x;
            if (pos.x < minX) minX = pos.x;
            
            if (pos.y > maxY) maxY = pos.y;
            if (pos.y < minY) minY = pos.y;
        }
        
        velocity *= 1 / size;
        
        double xDiff = maxX - minX;
        double yDiff = maxY - minY;
        radius_ = sqrt(xDiff * xDiff + yDiff * yDiff);
    }
    
    void initBodyDef(b2World *world)
    {
        bodyDef = new b2BodyDef();
        bodyDef->type = b2_dynamicBody;
        bodyDef->position.Set(this->center.x, this->center.y);
        
        body = world->CreateBody(bodyDef);
        initFixtureDef();
    }
    
    void initFixtureDef()
    {
        this->fixtureDef = new b2FixtureDef();
        fixtureDef = new b2FixtureDef();
        fixtureDef->density = 20.0;
        fixtureDef->friction = 0.0;
        fixtureDef->restitution = 0.0;
        fixtureDef->shape = new b2CircleShape(radius_);
        fixture = body->CreateFixture(fixtureDef);
    }
    
    b2Vec2 getPosition()const{return this->body->GetPosition();}
    b2Vec2 getVelocity()const{return this->body->GetLinearVelocity();}
    
    b2FixtureDef *fixtureDef;
    b2Fixture *fixture;
    
    b2Body *body;
    b2BodyDef *bodyDef;

    
//    void dispatch(double delta)
//    {
//        int size = allNodes.size();
//        for (int i = 0; i < size; i++)
//        {
//            Agent *ai = allNodes[i];
//            ai->body->ApplyLinearImpulse(force * delta, ai->getPosition());
//        }
//    }
};

#endif
