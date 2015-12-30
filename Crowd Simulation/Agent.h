//
//  Agent.h
//  Crowd Simulation
//
//  Created by z on 15-5-14.
//  Copyright (c) 2015年 SatanWoo. All rights reserved.
//

#ifndef Crowd_Simulation_Agent_h
#define Crowd_Simulation_Agent_h

#include <vector>
#include "Box2D.h"
#include "RVOTree.h"
#include "Definition.h"

using namespace RVO;
using namespace std;

class Agent {
public:
    b2Vec2 pos;
    b2Vec2 continuumForce;
    b2Vec2 flockForce;
    
    b2Vec2 goal;
    
    int group;
    int maxNeighbours;
    int preferVelocity;
    
    static float32 MAX_FORCE;
    static float32 MAX_SPEED;
    
    static float32 RADIUS;
    static float32 MIN_SEPARATION;
    
    // We'll move away from anyone nearer than this
    
    static float32 MAX_COHESION;
    static float32 MAX_FORCE_SQUARED;
    static float32 MAX_SPEED_SQUARED;
    
    Agent()
    {
        this->group = 0;
        this->pos = b2Vec2_zero;
        
        radius_ = Agent::RADIUS;
        maxSpeed_ = Agent::MAX_SPEED;
        
        neighborDist_ = 3;
    }

    Agent(b2Vec2 pos, int group)
    {
        this->group = group;
        this->pos = pos;
        
        radius_ = Agent::RADIUS;
        maxSpeed_ = Agent::MAX_SPEED;
        
        neighborDist_ = 3;
    }
    
    b2Vec2 getPosition()const{return this->body->GetPosition();}
    b2Vec2 getVelocity()const{return this->body->GetLinearVelocity();}
    
    void initBodyDef(b2World *world)
    {
        bodyDef = new b2BodyDef();
        bodyDef->type = b2_dynamicBody;
        bodyDef->position.Set(this->pos.x, this->pos.y);
        
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
        fixtureDef->shape = new b2CircleShape(Agent::RADIUS);
        fixture = body->CreateFixture(fixtureDef);
    }
    
    b2FixtureDef *fixtureDef;
    b2Fixture *fixture;
    
    b2Body *body;
    b2BodyDef *bodyDef;
    
#pragma mark - Neighbours
    RVOTree *tree;
    
    std::vector<std::pair<float, Agent *>> agentNeighbors_;
    
    size_t ID_;
    size_t maxNeighbors_;
    float32 maxSpeed_;
    float32 radius_;
    float32 neighborDist_;
    
    void computeNeighbors(HashMap& map)
    {
        float rangeSq = radius_ * radius_;
        
        agentNeighbors_.clear();
        
        if (maxNeighbors_ > 0)
        {
            rangeSq = neighborDist_ * neighborDist_;
            tree->computeAgentNeighbors(map, this, rangeSq);
        }
    }
    
    void insertAgentNeighbor(HashMap& map, Agent* agent, float &rangeSq)
    {
        if (this != agent && this->group == agent->group)
        {
            const float distSq = (getPosition() - agent->getPosition()).LengthSquared();
            
            if (distSq < rangeSq)
            {
                if (agentNeighbors_.size() < maxNeighbors_)
                {
                    map.insert(make_pair(agent->ID_, true));
                    agentNeighbors_.push_back(std::make_pair(distSq, agent));
                }
                
                size_t i = agentNeighbors_.size() - 1;
                
                while (i != 0 && distSq < agentNeighbors_[i - 1].first)
                {
                    agentNeighbors_[i] = agentNeighbors_[i - 1];
                    --i;
                }
                
                agentNeighbors_[i] = std::make_pair(distSq, agent);
                
                if (agentNeighbors_.size() == maxNeighbors_)
                {
                    rangeSq = agentNeighbors_.back().first;
                }
            }
        }
    }
};

#endif
