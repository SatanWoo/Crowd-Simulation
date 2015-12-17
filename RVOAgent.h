/*
 * Agent.h
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

#ifndef RVO_AGENT_H_
#define RVO_AGENT_H_
/**
 * \file       Agent.h
 * \brief      Contains the Agent class.
 */

#include "Definitions.h"
#include "Particle.h"
#include <Box2D/Box2D.h>
#include <unordered_map>
#include <vector>

namespace RVO {
    class RVOKDTree;
    
    struct RVOAgent : public Particle {
        b2Vec2 prefVelo;

        size_t maxNeighbours;

        typedef std::pair<float, const RVOAgent*> AgentDistanceMap;
        typedef std::vector<AgentDistanceMap> Neighours;
        
        Neighours neighbours;
        
        RVOKDTree *virtualTree;
        /**
         * \brief      Constructs an agent instance.
         * \param      sim             The simulator instance.
         */
        RVOAgent(b2Vec2 pos = b2Vec2_zero, b2Vec2 velocity = b2Vec2_zero, b2Vec2 pref = b2Vec2(4, 4),
                 float32 radius = 0.15, size_t maxNeighbours = 10, size_t ID = 0);
        RVOAgent(const RVOAgent& agent);
        
        void render(double gridSize);
        
        typedef std::unordered_map<size_t, bool> AgentsPool;
        
        void computeNeighbours(AgentsPool &pool);
        void insertAgentNeighbours(const RVOAgent* agent, float &range, AgentsPool& pool);
        
        b2Vec2 getPosition()const{return this->body->GetPosition();}
        b2Vec2 getVelocity()const{return this->body->GetLinearVelocity();}
        
        void initBodyDef(b2World *world);
        void initFixtureDef();
        
        // Box2D
        b2FixtureDef *fixtureDef;
        b2Fixture *fixture;
        
        b2Body *body;
        b2BodyDef *bodyDef;
    };
}

#endif /* RVO_AGENT_H_ */
