//
//  Particle.h
//  Crowd Simulation
//
//  Created by z on 15/12/17.
//  Copyright © 2015年 SatanWoo. All rights reserved.
//

#ifndef Particle_h
#define Particle_h

#include "Box2D.h"

struct Particle
{
    // Common Properties
    double radius;
    size_t ID;
    size_t group;
    
    b2Vec2 goal;
    b2Vec2 velocity;
    b2Vec2 pos;
    
    b2Vec2 continuumForce;
    b2Vec2 flockForce;
    
    virtual ~Particle()
    {
        
    }
    
    double maxCohesion()const
    {
        return MAX_COHESION;
    }
    
    double maxForce()const
    {
        return MAX_FORCE;
    }
    
    double maxSpeed()const
    {
        return MAX_SPEED;
    }
    
    double minSpearation()const
    {
        return MIN_SPEARATION;
    }
    
    double maxSpeedSquared()const
    {
        return MAX_FORCE_SQUARED;
    }
    
    double maxForceSquared()const
    {
        return MAX_FORCE_SQUARED;
    }
    
    b2Vec2 steeringForce(b2Vec2& direction)
    {
        b2Vec2 desiredVelocity = direction * maxSpeed();
        
        //最好的速度
        b2Vec2 velocityChange = desiredVelocity - this->velocity;
        
        // 转换成对着该方向运动的驱动力
        return velocityChange * (maxForce() / maxSpeed());
    }
    
    b2Vec2 steeringForce(b2Vec2& direction)const
    {
        b2Vec2 desiredVelocity = direction * maxSpeed();
        
        //最好的速度
        b2Vec2 velocityChange = desiredVelocity - this->velocity;
        
        // 转换成对着该方向运动的驱动力
        return velocityChange * (maxForce() / maxSpeed());
    }
    
    constexpr static const double MAX_SPEED = 4.0;
    constexpr static const double MAX_FORCE = 20.0;
    constexpr static const double MAX_COHESION = 2.0;
    constexpr static const double MIN_SPEARATION = 0.6;
    constexpr static const double MAX_SPEED_SQUARED = MAX_SPEED * MAX_SPEED;
    constexpr static const double MAX_FORCE_SQUARED = MAX_FORCE * MAX_FORCE;
};


#endif /* Particle_h */
