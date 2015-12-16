/*
 * Agent.cpp
 * RVO2 Library
 *
 * Copyright (c) 2008-2013 University of North Carolina at Chapel Hill.
 * All rights reserved.
 *
 * Permission to use, copy, modify, and distribute this software and its
 * documentation for educational, research, and non-profit purposes, without
 * fee, and without a written agreement is hereby granted, provided that the
 * above copyright notice, this paragraph, and the following four paragraphs
 * appear in all copies.
 *
 * Permission to incorporate this software into commercial products may be
 * obtained by contacting the authors <geom@cs.unc.edu> or the Office of
 * Technology Development at the University of North Carolina at Chapel Hill
 * <otd@unc.edu>.
 *
 * This software program and documentation are copyrighted by the University of
 * North Carolina at Chapel Hill. The software program and documentation are
 * supplied "as is," without any accompanying services from the University of
 * North Carolina at Chapel Hill or the authors. The University of North
 * Carolina at Chapel Hill and the authors do not warrant that the operation of
 * the program will be uninterrupted or error-free. The end-user understands
 * that the program was developed for research purposes and is advised not to
 * rely exclusively on the program for any reason.
 *
 * IN NO EVENT SHALL THE UNIVERSITY OF NORTH CAROLINA AT CHAPEL HILL OR THE
 * AUTHORS BE LIABLE TO ANY PARTY FOR DIRECT, INDIRECT, SPECIAL, INCIDENTAL, OR
 * CONSEQUENTIAL DAMAGES, INCLUDING LOST PROFITS, ARISING OUT OF THE USE OF THIS
 * SOFTWARE AND ITS DOCUMENTATION, EVEN IF THE UNIVERSITY OF NORTH CAROLINA AT
 * CHAPEL HILL OR THE AUTHORS HAVE BEEN ADVISED OF THE POSSIBILITY OF SUCH
 * DAMAGE.
 *
 * THE UNIVERSITY OF NORTH CAROLINA AT CHAPEL HILL AND THE AUTHORS SPECIFICALLY
 * DISCLAIM ANY WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE AND ANY
 * STATUTORY WARRANTY OF NON-INFRINGEMENT. THE SOFTWARE PROVIDED HEREUNDER IS ON
 * AN "AS IS" BASIS, AND THE UNIVERSITY OF NORTH CAROLINA AT CHAPEL HILL AND THE
 * AUTHORS HAVE NO OBLIGATIONS TO PROVIDE MAINTENANCE, SUPPORT, UPDATES,
 * ENHANCEMENTS, OR MODIFICATIONS.
 *
 * Please send all bug reports to <geom@cs.unc.edu>.
 *
 * The authors may be contacted via:
 *
 * Jur van den Berg, Stephen J. Guy, Jamie Snape, Ming C. Lin, Dinesh Manocha
 * Dept. of Computer Science
 * 201 S. Columbia St.
 * Frederick P. Brooks, Jr. Computer Science Bldg.
 * Chapel Hill, N.C. 27599-3175
 * United States of America
 *
 * <http://gamma.cs.unc.edu/RVO2/>
 */

#include "RVOAgent.h"
#include "RVOKDTree.h"

namespace RVO {
    
    int RVOAgent::maxForce = 20;
    int RVOAgent::maxSpeed = 4;
    
    float32 RVOAgent::minSeparation = 0.6;
    int RVOAgent::maxCohesion = 2;
    
    int RVOAgent::maxForceSquared = RVOAgent::maxForce * RVOAgent::maxForce;
    int RVOAgent::maxSpeedSquared = RVOAgent::maxSpeed * RVOAgent::maxSpeed;
    
    RVOAgent::RVOAgent(b2Vec2 pos, b2Vec2 velocity, b2Vec2 pref,
                       float32 radius, size_t maxNeighbours, size_t ID)
    {
        this->pos = pos;
        this->velocity = velocity;
        this->prefVelo = pref;
        this->radius = radius;
        this->maxNeighbours = maxNeighbours;
        this->ID = ID;
    }
    
    RVOAgent::RVOAgent(const RVOAgent& agent)
    {
        this->pos = agent.pos;
        this->velocity = agent.velocity;
        this->prefVelo = agent.prefVelo;
        this->radius = agent.radius;
        this->maxNeighbours = agent.maxNeighbours;
        this->ID = agent.ID;
    }
    
    void RVOAgent::computeNeighbours()
    {
        float rangeSq = radius * radius;
        neighbours.clear();
        
        if (maxNeighbours > 0)
        {
            this->virtualTree->computeAgentNeighbors(this, rangeSq);
        }
    }
    
    void RVOAgent::initBodyDef(b2World *world)
    {
        bodyDef = new b2BodyDef();
        bodyDef->type = b2_dynamicBody;
        bodyDef->position.Set(this->pos.x, this->pos.y);
        
        body = world->CreateBody(bodyDef);
        initFixtureDef();
    }
    
    void RVOAgent::initFixtureDef()
    {
        this->fixtureDef = new b2FixtureDef();
        fixtureDef = new b2FixtureDef();
        fixtureDef->density = 20.0;
        fixtureDef->friction = 0.0;
        fixtureDef->restitution = 0.0;
        fixtureDef->shape = new b2CircleShape(radius);
        fixture = body->CreateFixture(fixtureDef);
    }

    void RVOAgent::insertAgentNeighbours(const RVOAgent* agent, float &rangeSq)
	{
		if (this != agent)
        {
            if (this->group != agent->group) return;
            
			const float distSq = absSq(getPosition() - agent->getPosition());

			if (distSq < rangeSq)
            {
				if (neighbours.size() < maxNeighbours)
                {
					neighbours.push_back(std::make_pair(distSq, agent));
				}

                size_t i = neighbours.size() - 1;
				while (i != 0 && distSq < neighbours[i - 1].first)
                {
					neighbours[i] = neighbours[i - 1];
					--i;
				}

				neighbours[i] = std::make_pair(distSq, agent);

				if (neighbours.size() == maxNeighbours)
                {
					rangeSq = neighbours.back().first;
				}
			}
		}
	}

	void RVOAgent::update()
	{
//		velocity_ = newVelocity_;
//		position_ += velocity_ * sim_->timeStep_;
	}
}
