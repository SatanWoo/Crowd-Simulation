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

#include <queue>

static int fourDir[4][2] = {{0, -1}, {1, 0}, {0, 1}, {-1, 0}};
static int eightDir[8][2] = {{0, -1}, {1, 0}, {0, 1}, {-1, 0}, {1, 1}, {1, -1}, {-1, -1}, {-1, 1}};

#pragma mark - Public Method
SimulationController::SimulationController(size_t count)
{
    this->count = count;
//    this->clusters.resize(count);
//    this->agents.resize(count);
}

SimulationController::~SimulationController()
{
    for (size_t i = 0; i < this->clusters.size(); ++i)
    {
        this->clusters[i] = NULL;
        delete  this->clusters[i];
    }
    this->clusters.clear();
    
    for (size_t i = 0; i < this->agents.size(); ++i)
    {
        this->agents[i] = NULL;
        delete this->agents[i];
    }
    this->agents.clear();
    
    this->unclusteredAgents.clear();
    
    delete map;
    map = NULL;
    
    if (tree)
    {
        delete tree;
        tree = NULL;
    }
}

void SimulationController::simulate()
{
    continuumSimulation();
    flockSimulation();
    
//    if (this->mode == VirtualNode)
//    {
//        clusterSimulation();
//    }
    
    clusterSimulation();
    
    // Iteration times
    this->world->Step(this->timeStep, 10, 10);
    this->world->ClearForces();
}

void SimulationController::render()
{
    this->map->render();
    for (int i = 0; i < agents.size(); ++i)
    {
        RVOAgent *agent = agents[i];
        agent->render(MapController::MapGridSize);
    }
}

void SimulationController::init(int w, int h)
{
    static unsigned long long int usedID = 0;
    
    unsigned long long int size = w * h;
    
    srand((unsigned)time(NULL));
    
    this->map = new MapController(w, h);
    
    this->map->setMapSize(w, h);
    
    flowFlied.resize(size);
    avgVelocityField.resize(size);
    
    densityField.resize(size);
    potentialField.resize(size);
    discomfortField.resize(size);
    
    speedField.resize(size);
    unitCostField.resize(size);
    
    this->world = new b2World(b2Vec2_zero, true);
    
    for (int yPos = 1; yPos < 2; ++yPos)
    {
        for (int i = 0 ; i < 1; i++)
        {
            RVOAgent *agent = new RVOAgent(b2Vec2(i % 3, yPos));
            agent->group = 0;
            agent->ID = usedID++;
            agent->goal = b2Vec2(w - 2, h / 2.0);
            agent->initBodyDef(world);
            
            this->agents.push_back(agent);
        }
    }
    
    for (int yPos = 1; yPos < 2; yPos++)
    {
        for (int i =0 ; i < 1; i++)
        {
            RVOAgent *agent = new RVOAgent(b2Vec2(w - (i + 1) % 3, yPos));
            agent->group = 1;
            agent->ID = usedID++;
            agent->goal = b2Vec2(1, h / 2.0);
            agent->initBodyDef(world);
            
            this->agents.push_back(agent);
        }
    }
}

void SimulationController::stop()
{
    // Do customization refresh
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
    
    for (int i = 0; i < clusters.size(); ++i)
    {
        potentialField = vector<float>(this->map->getWidth() * this->map->getHeight(), FLT_MAX);
        buildPotentialField(clusters[i]->goal);
        buildFlowField();
        
        clusters[i]->continuumForce = forceFromFlowField(clusters[i]);
    }
}

void SimulationController::flockSimulation()
{
    for (int i = 0; i < clusters.size(); ++i)
    {
        Cluster *cluster = clusters[i];
        
        b2Vec2 sep = forceFromSeparation(cluster);
        b2Vec2 alg = forceFromAlignment(cluster);
        b2Vec2 coh = forceFromCohesion(cluster);
        
        cluster->flockForce = sep * relation.x + alg * relation.y + coh * relation.z;
        cluster->center = cluster->center + (cluster->continuumForce + cluster->flockForce) * this->timeStep;
    }
}

void SimulationController::clusterSimulation()
{
    this->unclusteredAgents.clear();
    // First Use KDTree to check
    for (int i = 0; i < agents.size(); i++)
    {
        agents[i]->computeNeighbours(this->unclusteredAgents);
    }
}

#pragma mark - Continuum Step
void SimulationController::resetContinnum()
{
    densityField.clear();
    discomfortField.clear();
    avgVelocityField.clear();
    
    // Obstacles.
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
    float32 densityMin = 0.5;
    float32 densityMax = 0.8;
    
    //Weights for formula (4) on page 4
    float32 lengthWeight = 1;
    float32 timeWeight = 1;
    float32 discomfortWeight = 1;
    
    for (int i = 0; i < this->map->getWidth(); i++) {
        for (int j = 0; j < this->map->getHeight(); j++) {
            
            //foreach direction we can leave that cell
            for (int dir = 0; dir < 4; dir++) {
                int targetX = i + fourDir[dir][0];
                int targetY = j + fourDir[dir][1];
                
                if (!this->map->isInMap(targetX, targetY)) {
                    speedField[index(i, j)].value[dir] = FLT_MAX;
                    continue;
                }
                
                float32 veloX = fourDir[dir][0] * avgVelocityField[index(targetX, targetY)].x;
                float32 veloY = fourDir[dir][1] * avgVelocityField[index(targetX, targetY)].y;
                
                //Get the only speed value as one will be zero
                // this is like speedVecX != 0 ? : speedVecX : speedVecY
                float32 flowSpeed = fourDir[dir][0] != 0 ? veloX : veloY;
                
                float32 density = densityField[index(targetX, targetY)];
                
                //cout << "X " << targetX << " Y " << targetY << " " << densityField[targetX][targetY] << endl;
                
                float32 discomfort = discomfortField[index(targetX, targetY)];
                
                if (density >= densityMax) {
                    speedField[index(i, j)].value[dir] = flowSpeed;
                } else if (density <= densityMin) {
                    speedField[index(i, j)].value[dir] = RVOAgent::maxSpeed;
                } else {
                    //medium speed
                    speedField[index(i, j)].value[dir] = RVOAgent::maxSpeed - (density - densityMin) / (densityMax - densityMin) * (4 - flowSpeed);
                }
                
                //we're going to divide by speed later, so make sure it's not zero
                float32 speed = speedField[index(i, j)].value[dir];
                float32 threshold = 0.001;
                speedField[index(i, j)].value[dir] = max(threshold, speed);
                
                //Work out the cost to move in to the destination cell
                unitCostField[index(i, j)].value[dir] = (speedField[index(i, j)].value[dir] * lengthWeight + timeWeight + discomfortWeight * discomfort) / speedField[index(i, j)].value[dir];
                
                //cout << "X:" << i << "Y:" << j << "I:" << dir << " " <<costField[i][j].value[dir] << endl;
            }
        }
    }
}

void SimulationController::buildPotentialField(const b2Vec2 &goal)
{
    static vector<bool> visited(this->map->getWidth() * this->map->getHeight(), false);
    visited.clear();
    
    int candidatesCount = 0;
    
    CostNode desNode;
    desNode.cost = 0;
    desNode.point = goal;
    
    priority_queue<CostNode> pQueue;
    pQueue.push(desNode);
    
    while (!pQueue.empty())
    {
        candidatesCount++;
        CostNode at = pQueue.top();
        pQueue.pop();
        
        int x = at.point.x;
        int y = at.point.y;
        
        if (potentialField[index(x, y)] >= at.cost && !visited[index(x, y)])
        {
            potentialField[index(x, y)] = at.cost;
            visited[index(x, y)] = true;
            
            for (int i = 0; i < 4; i++)
            {
                int toX = x + fourDir[i][0];
                int toY = y + fourDir[i][1];
                if (this->map->isInMap(toX, toY)) {
                    //Cost to go from our target cell to the start
                    //Our cost + cost of moving from the target to us
                    float32 toCost = at.cost + unitCostField[index(toX, toY)].value[(i + 2) % 4];
                    
                    //If we present a better path, overwrite the cost and queue up going to that cell
                    if (toCost < potentialField[index(toX, toY)])
                    {
                        potentialField[index(toX, toY)] = toCost;
                        visited[index(x, y)] = false;
                        
                        CostNode toP(b2Vec2(toX, toY), toCost);
                        pQueue.push(toP);
                    }
                }
            }
        }
    }
}

void SimulationController::buildFlowField()
{
    flowFlied.clear();
    
    for (int i = 0; i < this->map->getWidth(); ++i)
    {
        for (int j = 0; j < this->map->getHeight(); ++j)
        {
            if (potentialField[index(i, j)] == FLT_MAX) continue;
            
            bool isMinFound = false;
            b2Vec2 min = b2Vec2_zero;
            float32 minDist = FLT_MAX;
            
            for (int d = 0; d < 8; d++)
            {
                if (this->map->isInMap(i + eightDir[d][0], j + eightDir[d][1]))
                {
                    float32 dist = potentialField[index(i + eightDir[d][0], j + eightDir[d][1])];
                    
                    if (dist < minDist)
                    {
                        min.x = eightDir[d][0];
                        min.y = eightDir[d][1];
                        minDist = dist;
                        
                        isMinFound = true;
                    }
                }
            }
            
            if (isMinFound)
            {
                flowFlied[index(i, j)].Set(min.x, min.y);
                flowFlied[index(i, j)].Normalize();
            }
        }
    }
}

void SimulationController::buildDensityField(int x, int y, const b2Vec2 &vec, float weight)
{
    int i = index(x, y);
    this->densityField[i] += weight;
    avgVelocityField[i] += vec * weight;
}

#pragma mark - Force
b2Vec2 SimulationController::forceFromFlowField(Cluster *cluster)
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

b2Vec2 SimulationController::forceFromSeparation(Cluster *cluster)
{
    b2Vec2 totalForce = b2Vec2_zero;
    
    int neighboursCount = 0;
    
    for (int i = 0; i < clusters.size(); i++) {
        Cluster* other = clusters[i];
        if (other != cluster) {
            float32 distance = B2Vec2DHelper::distanceTo(other->center, cluster->center);
            if (distance < RVOAgent::minSeparation && distance > 0) {
                b2Vec2 pushForce = cluster->center - other->center;
                float32 length = pushForce.Normalize(); //Normalize returns the original length
                float32 r = cluster->radius + other->radius;

                totalForce += pushForce * (1 - ((length - r) / (RVOAgent::minSeparation - r)));//agent.minSeparation)));
                neighboursCount++;
            }
        }
    }

    if (neighboursCount == 0) {
        return totalForce; //Zero
    }

    return totalForce * (RVOAgent::maxForce / neighboursCount);
}

b2Vec2 SimulationController::forceFromCohesion(Cluster *cluster)
{
    //agent.position().Copy();
    b2Vec2 centerOfMass = b2Vec2_zero;
    int neighboursCount = 0;

    for (int i = 0; i < clusters.size(); i++)
    {
        Cluster *other = clusters[i];
        if (other != cluster && other->group == cluster->group)
        {
            float32 distance = B2Vec2DHelper::distanceTo(cluster->center, other->center);
            if (distance < RVOAgent::maxCohesion)
            {
                //sum up the position of our neighbours
                centerOfMass += other->center;
                neighboursCount++;
            }
        }
    }

    if (neighboursCount == 0) {
        return b2Vec2_zero;
    }

    //Get the average position of ourself and our neighbours
    centerOfMass *= (1 / neighboursCount);

    //seek that position
    return forceFromSeek(cluster, centerOfMass);
}

b2Vec2 SimulationController::forceFromAlignment(Cluster *cluster)
{
    b2Vec2 averageHeading = b2Vec2_zero;
    int neighboursCount = 0;
    
    //for each of our neighbours (including ourself)
    for (int i = 0; i < clusters.size(); i++) {
        Cluster *other = clusters[i];
        float32 distance = B2Vec2DHelper::distanceTo(cluster->center, other->center);
        //That are within the max distance and are moving
        if (distance < RVO::RVOAgent::maxCohesion && other->velocity.Length() > 0 && cluster->group == other->group) {
            //Sum up our headings
            b2Vec2 head = other->velocity;
            head.Normalize();
            averageHeading += head;
            neighboursCount++;
        }
    }
    
    if (neighboursCount == 0) {
        return averageHeading; //Zero
    }
    
    //Divide to get the average heading
    averageHeading *= (1 / neighboursCount);
    
    //Steer towards that heading
    return cluster->steeringForce(averageHeading);
}

b2Vec2 SimulationController::forceFromSeek(Cluster *cluster, b2Vec2& dest)
{
    if (dest.x == cluster->center.x && dest.y == cluster->center.y) {
        return b2Vec2_zero;
    }
    
    b2Vec2 desired = dest - cluster->center;
    
    desired *= (RVOAgent::maxSpeed / desired.Length());

    b2Vec2 velocityChange = desired - cluster->velocity;
    return velocityChange * (RVOAgent::maxForce / RVOAgent::maxSpeed);
}

#pragma mark - Helper
inline int SimulationController::index(int x, int y)const
{
    return x * this->map->getWidth() + y;
}