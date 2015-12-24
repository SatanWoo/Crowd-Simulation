//
//  Agent.h
//  Crowd Simulation
//
//  Created by z on 15-5-14.
//  Copyright (c) 2015年 SatanWoo. All rights reserved.
//

#ifndef Crowd_Simulation_Agent_h
#define Crowd_Simulation_Agent_h

#include "Box2D.h"
#include <vector>
#include "RVOTree.h"

using namespace RVO;

struct Agent {
    b2Vec2 pos;
    b2Vec2 ff;
    b2Vec2 force;
    int group;
    
    std::vector<int> neighbours;
    
    static int maxForce; //rate of acceleration
    static int maxSpeed; //grid squares / second
    
    static float32 radius;
    static float32 minSeparation; // We'll move away from anyone nearer than this
    
    static int maxCohesion; //We'll move closer to anyone within this bound
    
    static int maxForceSquared;
    static int maxSpeedSquared;
    
    static int maxNeighbours;
    static int preferVelocity;
    
    Agent(b2Vec2 pos, int group){
        this->group = group;
        this->pos = pos;
        
        radius_ = Agent::radius;
        maxSpeed_ = Agent::maxSpeed;
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
        fixtureDef->shape = new b2CircleShape(Agent::radius);
        fixture = body->CreateFixture(fixtureDef);
    }
    
    b2FixtureDef *fixtureDef;
    b2Fixture *fixture;
    
    b2Body *body;
    b2BodyDef *bodyDef;
    
#pragma mark - Neighbours
    RVOTree *tree;
    
    std::vector<std::pair<float, Agent *>> agentNeighbors_;
    
    size_t maxNeighbors_;
    float maxSpeed_;
    float radius_;
    float neighborDist_;
    
    size_t ID_;
    
    void computeNeighbors()
    {
        float rangeSq = radius_ * radius_;
        
        agentNeighbors_.clear();
        
        if (maxNeighbors_ > 0) {
            rangeSq = neighborDist_ * neighborDist_;
            tree->computeAgentNeighbors(this, rangeSq);
        }
    }
    
    void insertAgentNeighbor(Agent* agent, float &rangeSq)
    {
        if (this != agent)
        {
            const float distSq = (pos - agent->pos).LengthSquared();
            
            if (distSq < rangeSq)
            {
                if (agentNeighbors_.size() < maxNeighbors_)
                {
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
