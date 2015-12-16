//
//  SimulationController.cpp
//  Crowd Simulation
//
//  Created by z on 15/12/16.
//  Copyright © 2015年 SatanWoo. All rights reserved.
//

#include "SimulationController.h"
#include "MapController.h"
#include "B2Vec2DHelper.h"
#include "CostNode.h"
#include "Render.h"

#pragma mark - Public Method
SimulationController::SimulationController(size_t count)
{
    this->count = count;
    this->clusters.resize(count);
    this->agents.resize(count);
}

SimulationController::~SimulationController()
{
    for (size_t i = 0; i < this->clusters.size(); ++i)
    {
        delete  this->clusters[i];
    }
    this->clusters.clear();
    
    for (size_t i = 0; i < this->agents.size(); ++i)
    {
        delete this->agents[i];
    }
    this->agents.clear();
    
    this->unclusteredAgents.clear();
}

void SimulationController::simulate()
{
    continuumSimulation();
    flockSimulation();
    
    if (this->mode == VirtualNode)
    {
        clusterSimulation();
    }
}

void SimulationController::render()
{
    Render::render(this->map);
}

void SimulationController::init(int w, int h)
{
    unsigned long long int size = w * h;
    
    this->map->setMapSize(w, h);
    
    flowFlied.resize(size);
    avgVelocityField.resize(size);
    
    densityField.resize(size);
    potentialField.resize(size);
    discomfortField.resize(size);
    
    speedField.resize(size);
    unitCostField.resize(size);
    
    
//    for (int ; <#condition#>; <#increment#>) {
//        <#statements#>
//    }
//    
//    for (int yPos = 1; yPos < m_iHeight - 1; yPos++)
//    {
//        for (int i = 0 ; i < 30; i++)
//        {
//            RVO::RVOAgent p1(b2Vec2(i % 3, yPos), 0);
//            p1.initBodyDef(world);
//            agents.push_back(p1);
//        }
//    }
//    
//    for (int yPos = 1; yPos < m_iHeight - 1; yPos++)
//    {
//        for (int i =0 ; i < 30; i++)
//        {
//            //            Agent p1(b2Vec2(m_iWidth - (i + 1) % 3, yPos), 1);
//            //            p1.initBodyDef(world);
//            //            agents.push_back(p1);
//        }
//    }
    
//    for (size_t i = 0; i < this->count; ++i)
//    {
//        agents[i] = 
//    }
}

void SimulationController::stop()
{
    
}

void SimulationController::enableVirtualNodeMode()
{
    this->mode = VirtualNode;
}

#pragma mark - Protected Method
void SimulationController::continuumSimulation()
{
    resetContinnum();
    calculateDensityAndAverageSpeed();
    calculateUnitCostField();
    
    for (int i = 0; i < clusters.size(); i++)
    {
        potentialField.clear();
        buildPotentialField(clusters[i]->goal);
        buildFlowField();
        
        clusters[i]->fl = forceFromFlowField(clusters[i]);
    }
    
    for (int group = 0; group <= 1; group++) //foreach group
    {
        potentialField.clear();
        
        //Construct the potential
        buildFlowField();
        ccPotentialFieldEikonalFill(destinationPoints[group]);
        //Compute the gradient
        //ccCalculatePotentialFieldGradient();
        ccGenerateFlowField(); //TODO: This does not use the way of calculating described in the paper (I think)
        
        //(use these for steering later)
        for (int i = virtualNodes.size() - 1; i >= 0; i--) {
            if (virtualNodes[i].group == group) {
                virtualNodes[i].ff = steeringBehaviourFlowField(virtualNodes[i]);
            }
        }
    }
}

void SimulationController::flockSimulation()
{
    
}

void SimulationController::clusterSimulation()
{
    
}

#pragma mark - Continuum Step
void SimulationController::resetContinnum()
{
    densityField.clear();
    discomfortField.clear();
    avgVelocityField.clear();
//    for (int i = obstacles.size() - 1; i >= 0; i--) {
//        b2Vec2 o = obstacles[i];
//        int x = o.x, y = o.y;
//        discomfortField[x][y] = INT_MAX;
//    }
}

void SimulationController::calculateDensityAndAverageSpeed()
{
    float32 perAgentDensity = 1.0;

    for (int i = 0; i < clusters.size(); i++)
    {
        //Agent &pi = agents[i];
        
        const Cluster *cluster = clusters[i];

        b2Vec2 floor = B2Vec2DHelper::floorV(cluster->center);
        float32 xWeight = cluster->center.x - floor.x;
        float32 yWeight = cluster->center.y - floor.y;

        //top left
        if (this->map->isInMap(floor.x, floor.y)) {
            buildDensityField(floor.x, floor.y, cluster->velocity, perAgentDensity * (1 - xWeight) * (1 - yWeight));
        }
        //top right
        if (this->map->isInMap(floor.x + 1, floor.y)) {
            buildDensityField(floor.x + 1, floor.y, cluster->velocity, perAgentDensity * (xWeight) * (1 - yWeight));
        }
        //bottom left
        if (this->map->isInMap(floor.x, floor.y + 1)) {
            buildDensityField(floor.x, floor.y + 1, cluster->velocity, perAgentDensity * (1 - xWeight) * (yWeight));
        }
        //bottom right
        if (this->map->isInMap(floor.x + 1, floor.y + 1)) {
            buildDensityField(floor.x + 1, floor.y + 1, cluster->velocity, perAgentDensity * (xWeight) * (yWeight));
        }
    }
}

void SimulationController::calculateUnitCostField()
{
    
}

void SimulationController::buildDensityField(int x, int y, const b2Vec2 &vec, float weight)
{
    int i = index(x, y);
    this->densityField[i] += weight;
    avgVelocityField[i] += vec * weight;
}

#pragma mark - Force
b2Vec2 SimulationController::forceFromFlowField(const Cluster *cluster)
{
    b2Vec2 floor = B2Vec2DHelper::floorV(cluster->center);
    
    int x = floor.x;
    int y = floor.y;
    
    b2Vec2 f00 = this->map->isInMap(x, y) ? this->flowFlied[index(x, y)] : b2Vec2_zero;
    b2Vec2 f01 = this->map->isInMap(x, y + 1) ? this->flowFlied[index(x, y + 1)] : b2Vec2_zero;
    b2Vec2 f10 = this->map->isInMap(x + 1, y) ? this->flowFlied[index(x + 1, y)] : b2Vec2_zero;
    b2Vec2 f11 = this->map->isInMap(x + 1, y + 1) ? this->flowFlied[index(x + 1, y + 1)] : b2Vec2_zero;
    
    // 水平方向插值
    float32 xWeight = cluster->center.x - floor.x;
    
    b2Vec2 top = f00 * (1 - xWeight) +  f10 * xWeight;
    b2Vec2 bottom = f01 * (1 - xWeight) + f11 * xWeight;
    
    // 竖直方向插值
    float32 yWeight = cluster->center.y - floor.y;
    
    // 基于水平和竖直方向上的插值，计算得出新的运动方向
    b2Vec2 desiredDirection = top * (1 - yWeight) + bottom * yWeight;
    desiredDirection.Normalize();
    
    // 处于网格中心就不会产生流体力
    if (isnan(desiredDirection.LengthSquared())) {
        return b2Vec2_zero;
    }
    
    return cluster->steeringForce(desiredDirection);
}

#pragma mark - Helper
inline int SimulationController::index(int x, int y)const
{
    return x * this->map->getWidth() + y;
}