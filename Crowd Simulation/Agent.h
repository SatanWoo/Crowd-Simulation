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

struct Agent {
    b2Vec2 pos;
    int group;
    
    static int maxForce; //rate of acceleration
    static int maxSpeed; //grid squares / second
    
    static float32 radius;
    static float32 minSeparation; // We'll move away from anyone nearer than this
    
    static int maxCohesion; //We'll move closer to anyone within this bound
    
    static int maxForceSquared;
    static int maxSpeedSquared;
    
    Agent(b2Vec2 pos, int group){
        this->group = group;
        this->pos = pos;
    }
    
    b2Vec2 getPosition()const{return this->body->GetPosition();}
    b2Vec2 getVelocity()const{return this->body->GetLinearVelocity();}
    
    void initBodyDef(b2World *world);
    void initFixtureDef();
    
private:
    b2FixtureDef *fixtureDef;
    b2Fixture *fixture;
    
    b2Body *body;
    b2BodyDef *bodyDef;
};

void Agent::initBodyDef(b2World *world)
{
    bodyDef = new b2BodyDef();
    bodyDef->type = b2_dynamicBody;
    bodyDef->position.Set(this->pos.x, this->pos.y);
    
    body = world->CreateBody(bodyDef);
    initFixtureDef();
}

void Agent::initFixtureDef()
{
    this->fixtureDef = new b2FixtureDef();
    fixtureDef = new b2FixtureDef();
    fixtureDef->density = 20.0;
    fixtureDef->friction = 0.0;
    fixtureDef->restitution = 0.0;
    fixtureDef->shape = new b2CircleShape(Agent::radius);
    fixture = body->CreateFixture(fixtureDef);
}

int Agent::maxForce = 20;
int Agent::maxSpeed = 4;
float32 Agent::radius = 0.23;
float32 Agent::minSeparation = 0.8;
int Agent::maxCohesion = 2;
int Agent::maxForceSquared = Agent::maxForce * Agent::maxForce;
int Agent::maxSpeedSquared = Agent::maxSpeed * Agent::maxSpeed;

#endif
