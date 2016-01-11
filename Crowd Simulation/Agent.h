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
    b2Vec2 ff;
    b2Vec2 force;
    int group;
    
    float32 maxForce;
    float32 maxSpeed;
    
    float32 radius;
    float32 neighbourRadius;
    
    float32 maxForceSquared;
    float32 maxSpeedSquared;
    
    Agent(b2Vec2 pos, int group){
        this->group = group;
        this->pos = pos;
        
        maxForce = 50;
        maxSpeed = 4;
        radius = 0.23;
        
        neighbourRadius = 3;
        
        maxForceSquared = maxForce * maxForce;
        maxSpeedSquared = maxSpeed * maxSpeed;
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
        fixtureDef->shape = new b2CircleShape(this->radius);
        fixture = body->CreateFixture(fixtureDef);
    }
    
    b2FixtureDef *fixtureDef;
    b2Fixture *fixture;
    
    b2Body *body;
    b2BodyDef *bodyDef;
};

#endif
