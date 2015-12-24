/*
 * KdTree.cpp
 * RVO2 Library
 *
 * Copyright (c) 2008-2010 University of North Carolina at Chapel Hill.
 * All rights reserved.
 *
 * Permission to use, copy, modify, and distribute this software and its
 * documentation for educational, research, and non-profit purposes, without
 * fee, and without a written agreement is hereby granted, provided that the
 * above copyright notice, this paragraph, and the following four paragraphs
 * appear in all copies.
 *
 * Permission to incorporate this software into commercial products may be
 * obtained by contacting the Office of Technology Development at the University
 * of North Carolina at Chapel Hill <otd@unc.edu>.
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

#include "RVOTree.h"
#include "Agent.h"

namespace RVO {
    RVOTree::RVOTree(size_t count):count_(count){}

	RVOTree::~RVOTree()
	{}

    void RVOTree::buildAgentTree(const std::vector<Agent>& agents)
	{
        agents_.clear();
        
        for (size_t i = agents_.size(); i < agents.size(); ++i)
        {
            Agent agent = agents[i];
            agents_.push_back(&agent);
        }
        
		if (!agents_.empty()) {
			buildAgentTreeRecursive(0, agents_.size(), 0);
		}
	}
    
    void RVOTree::buildAgentTreeRecursive(size_t begin, size_t end, size_t node)
	{
		agentTree_[node].begin = begin;
		agentTree_[node].end = end;
		agentTree_[node].minX = agentTree_[node].maxX = agents_[begin]->getPosition().x;
		agentTree_[node].minY = agentTree_[node].maxY = agents_[begin]->getPosition().y;

		for (size_t i = begin + 1; i < end; ++i) {
			agentTree_[node].maxX = std::max(agentTree_[node].maxX, agents_[i]->getPosition().x);
			agentTree_[node].minX = std::min(agentTree_[node].minX, agents_[i]->getPosition().x);
			agentTree_[node].maxY = std::max(agentTree_[node].maxY, agents_[i]->getPosition().y);
			agentTree_[node].minY = std::min(agentTree_[node].minY, agents_[i]->getPosition().y);
		}

		if (end - begin > MAX_LEAF_SIZE)
        {
			/* No leaf node. */
			const bool isVertical = (agentTree_[node].maxX - agentTree_[node].minX > agentTree_[node].maxY - agentTree_[node].minY);
			const float splitValue = (isVertical ? 0.5f * (agentTree_[node].maxX + agentTree_[node].minX) : 0.5f * (agentTree_[node].maxY + agentTree_[node].minY));

			size_t left = begin;
			size_t right = end;

			while (left < right) {
				while (left < right && (isVertical ? agents_[left]->getPosition().x : agents_[left]->getPosition().y) < splitValue) {
					++left;
				}

				while (right > left && (isVertical ? agents_[right - 1]->getPosition().x : agents_[right - 1]->getPosition().y) >= splitValue) {
					--right;
				}

				if (left < right) {
					std::swap(agents_[left], agents_[right - 1]);
					++left;
					--right;
				}
			}

			if (left == begin) {
				++left;
				++right;
			}

			agentTree_[node].left = node + 1;
			agentTree_[node].right = node + 2 * (left - begin);

			buildAgentTreeRecursive(begin, left, agentTree_[node].left);
			buildAgentTreeRecursive(left, end, agentTree_[node].right);
		}
	}

	void RVOTree::computeAgentNeighbors(Agent* agent, float &rangeSq)
	{
		queryAgentTreeRecursive(agent, rangeSq, 0);
	}
    
	void RVOTree::queryAgentTreeRecursive(Agent* agent, float &rangeSq, size_t node)
	{
		if (agentTree_[node].end - agentTree_[node].begin <= MAX_LEAF_SIZE)
        {
            for (size_t i = agentTree_[node].begin; i < agentTree_[node].end; ++i)
            {
				agent->insertAgentNeighbor(agents_[i], rangeSq);
			}
		}
		else
        {
			const float distSqLeft = sqr(std::max(0.0f, agentTree_[agentTree_[node].left].minX - agent->getPosition().x)) + sqr(std::max(0.0f, agent->pos.x - agentTree_[agentTree_[node].left].maxX)) + sqr(std::max(0.0f, agentTree_[agentTree_[node].left].minY - agent->getPosition().y)) + sqr(std::max(0.0f, agent->getPosition().y - agentTree_[agentTree_[node].left].maxY));

			const float distSqRight = sqr(std::max(0.0f, agentTree_[agentTree_[node].right].minX - agent->getPosition().x)) + sqr(std::max(0.0f, agent->getPosition().x - agentTree_[agentTree_[node].right].maxX)) + sqr(std::max(0.0f, agentTree_[agentTree_[node].right].minY - agent->getPosition().y)) + sqr(std::max(0.0f, agent->getPosition().y - agentTree_[agentTree_[node].right].maxY));

			if (distSqLeft < distSqRight)
            {
				if (distSqLeft < rangeSq)
                {
					queryAgentTreeRecursive(agent, rangeSq, agentTree_[node].left);

					if (distSqRight < rangeSq)
                    {
						queryAgentTreeRecursive(agent, rangeSq, agentTree_[node].right);
					}
				}
			}
			else
            {
				if (distSqRight < rangeSq)
                {
					queryAgentTreeRecursive(agent, rangeSq, agentTree_[node].right);

					if (distSqLeft < rangeSq)
                    {
						queryAgentTreeRecursive(agent, rangeSq, agentTree_[node].left);
					}
				}
			}

		}
	}
}
