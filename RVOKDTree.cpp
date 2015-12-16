/*
 * RVOKDTree.cpp
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

#include "RVOKDTree.h"
#include "RVOAgent.h"

namespace RVO {
	RVOKDTree::RVOKDTree()
    {
    }
    
    RVOKDTree::RVOKDTree(const std::vector<RVOAgent *>& agents)
    {
        this->agents.assign(agents.begin(), agents.end());
    }

	RVOKDTree::~RVOKDTree()
	{
        this->agents.clear();
        this->agentTree.clear();
	}

	void RVOKDTree::buildAgentTree()
	{
        agentTree.resize(2 * agents.size() - 1);
        buildAgentTreeRecursive(0, agents.size(), 0);
	}

	void RVOKDTree::buildAgentTreeRecursive(size_t begin, size_t end, size_t nodeID)
	{
		agentTree[nodeID].begin = begin;
		agentTree[nodeID].end = end;
		agentTree[nodeID].minX = agentTree[nodeID].maxX = agents[begin]->getPosition().x;
		agentTree[nodeID].minY = agentTree[nodeID].maxY = agents[begin]->getPosition().y;

		for (size_t i = begin + 1; i < end; ++i) {
			agentTree[nodeID].maxX = std::max(agentTree[nodeID].maxX, agents[i]->getPosition().x);
			agentTree[nodeID].minX = std::min(agentTree[nodeID].minX, agents[i]->getPosition().x);
			agentTree[nodeID].maxY = std::max(agentTree[nodeID].maxY, agents[i]->getPosition().y);
			agentTree[nodeID].minY = std::min(agentTree[nodeID].minY, agents[i]->getPosition().y);
		}

		if (end - begin > MAX_LEAF_SIZE) {
			/* No leaf node. */
			const bool isVertical = (agentTree[nodeID].maxX - agentTree[nodeID].minX > agentTree[nodeID].maxY - agentTree[nodeID].minY);
			const float splitValue = (isVertical ? 0.5f * (agentTree[nodeID].maxX + agentTree[nodeID].minX) : 0.5f * (agentTree[nodeID].maxY + agentTree[nodeID].minY));

			size_t left = begin;
			size_t right = end;

			while (left < right) {
				while (left < right && (isVertical ? agents[left]->getPosition().x : agents[left]->getPosition().y) < splitValue) {
					++left;
				}

				while (right > left && (isVertical ? agents[right - 1]->getPosition().x : agents[right - 1]->getPosition().y) >= splitValue) {
					--right;
				}

				if (left < right) {
					std::swap(agents[left], agents[right - 1]);
					++left;
					--right;
				}
			}

			if (left == begin) {
				++left;
				++right;
			}

            // 2n, 2n + 1
			agentTree[nodeID].left = nodeID + 1;
			agentTree[nodeID].right = nodeID + 2 * (left - begin);

			buildAgentTreeRecursive(begin, left, agentTree[nodeID].left);
			buildAgentTreeRecursive(left, end, agentTree[nodeID].right);
		}
	}
    
    void RVOKDTree::computeAgentNeighbors(RVOAgent *agent, float &rangeSq)
    {
        queryAgentTreeRecursive(agent, rangeSq, 0);
    }

	void RVOKDTree::queryAgentTreeRecursive(RVOAgent* agent, float &rangeSq, size_t node)const
	{
		if (agentTree[node].end - agentTree[node].begin <= MAX_LEAF_SIZE)
        {
			for (size_t i = agentTree[node].begin; i < agentTree[node].end; ++i)
            {
				agent->insertAgentNeighbours(agents[i], rangeSq);
			}
		}
		else
        {
            size_t left = agentTree[node].left, right = agentTree[node].right;
            // 计算当前Agent的位置到Box4个顶点之前的距离之和。
			const float distSqLeft = sqr(std::max(0.0f, agentTree[left].minX - agent->getPosition().x)) +
                                     sqr(std::max(0.0f, agent->getPosition().x - agentTree[left].maxX)) +
                                     sqr(std::max(0.0f, agentTree[left].minY - agent->getPosition().y)) +
                                     sqr(std::max(0.0f, agent->getPosition().y - agentTree[left].maxY));

			const float distSqRight = sqr(std::max(0.0f, agentTree[right].minX - agent->getPosition().x)) +
                                      sqr(std::max(0.0f, agent->getPosition().x - agentTree[right].maxX)) +
                                      sqr(std::max(0.0f, agentTree[right].minY - agent->getPosition().y)) +
                                      sqr(std::max(0.0f, agent->getPosition().y - agentTree[right].maxY));

			if (distSqLeft < distSqRight)
            {
                // 离左子树近
				if (distSqLeft < rangeSq)
                {
					queryAgentTreeRecursive(agent, rangeSq, left);
                    
                    // 必要的回溯
					if (distSqRight < rangeSq) {
						queryAgentTreeRecursive(agent, rangeSq, right);
					}
				}
			}
			else
            {
                // 离右子树近
				if (distSqRight < rangeSq)
                {
					queryAgentTreeRecursive(agent, rangeSq, right);

					if (distSqLeft < rangeSq) {
						queryAgentTreeRecursive(agent, rangeSq, left);
					}
				}
			}
		}
	}
}
