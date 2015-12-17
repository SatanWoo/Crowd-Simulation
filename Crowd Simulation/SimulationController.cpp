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
        delete  this->clusters[i];
        this->clusters[i] = NULL;
    }
    this->clusters.clear();
    
    for (size_t i = 0; i < this->agents.size(); ++i)
    {
        delete this->agents[i];
        this->agents[i] = NULL;
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
    //flockSimulation();
    
//    if (this->mode == VirtualNode)
//    {
//        clusterSimulation();
//    }
    
    //clusterSimulation();
    
    for (int i = 0; i < agents.size(); ++i)
    {
        RVOAgent *agent = agents[i];
        agent->body ->ApplyLinearImpulse(agent->continuumForce * this->timeStep, agent->getPosition());
    }
    
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
        for (int i = 0 ; i < 1; i++)
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
    
    for (int i = 0; i < agents.size(); ++i)
    {
        potentialField = vector<float>(this->map->getWidth() * this->map->getHeight(), FLT_MAX);
        buildPotentialField(agents[i]->goal);
        buildFlowField();
        
        agents[i]->continuumForce = forceFromFlowField(agents[i]);
    }
}

void SimulationController::flockSimulation()
{
    for (int i = 0; i < agents.size(); ++i)
    {
        RVOAgent *agent = agents[i];
        
        b2Vec2 sep = forceFromSeparation(agents[i]);
        b2Vec2 alg = forceFromAlignment(agents[i]);
        b2Vec2 coh = forceFromCohesion(agents[i]);
        
        agent->flockForce = sep * relation.x + alg * relation.y + coh * relation.z + agent->continuumForce;
    
        
        float32 lengthSquared =  agent->flockForce.LengthSquared();
        if (lengthSquared > agent->maxForceSquared()) {
            agent->flockForce *= (agent->maxForceSquared() / sqrt(lengthSquared));
        }
        //agent->center = cluster->center + (cluster->continuumForce + cluster->flockForce) * this->timeStep;
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
    densityField.resize(this->map->getHeight() * this->map->getWidth());
    discomfortField.clear();
    discomfortField.resize(this->map->getHeight() * this->map->getWidth());
    avgVelocityField.clear();
    avgVelocityField.resize(this->map->getHeight() * this->map->getWidth());
    
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

    for (int i = 0; i < agents.size(); i++)
    {
        RVOAgent *agent = agents[i];

        b2Vec2 floor = B2Vec2DHelper::floorV(agent->getPosition());
        float32 xWeight = agent->getPosition().x - floor.x;
        float32 yWeight = agent->getPosition().y - floor.y;

        //top left
        if (this->map->isInMap(floor.x, floor.y)) {
            buildDensityField(floor.x, floor.y, agent->getVelocity(), perAgentDensity * (1 - xWeight) * (1 - yWeight));
        }
        //top right
        if (this->map->isInMap(floor.x + 1, floor.y)) {
            buildDensityField(floor.x + 1, floor.y, agent->getVelocity(), perAgentDensity * (xWeight) * (1 - yWeight));
        }
        //bottom left
        if (this->map->isInMap(floor.x, floor.y + 1)) {
            buildDensityField(floor.x, floor.y + 1, agent->getVelocity(), perAgentDensity * (1 - xWeight) * (yWeight));
        }
        //bottom right
        if (this->map->isInMap(floor.x + 1, floor.y + 1)) {
            buildDensityField(floor.x + 1, floor.y + 1, agent->getVelocity(), perAgentDensity * (xWeight) * (yWeight));
        }
    }
    
    for (int i = 0; i < this->map->getWidth(); ++i)
    {
        for (int j = 0; j < this->map->getHeight(); ++j)
        {
            b2Vec2& velocity = this->avgVelocityField[index(i, j)];
            float32 density = densityField[index(i, j)];
            if (density > 0)
            {
                velocity *= (1/density);
            }
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
    
    for (int i = 0; i < this->map->getWidth(); ++i)
    {
        for (int j = 0; j < this->map->getHeight(); ++j)
        {
            //foreach direction we can leave that cell
            for (int dir = 0; dir < 4; ++dir)
            {
                int targetX = i + fourDir[dir][0];
                int targetY = j + fourDir[dir][1];
                
                if (!this->map->isInMap(targetX, targetY))
                {
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
                
                if (density >= densityMax)
                {
                    speedField[index(i, j)].value[dir] = flowSpeed;
                }
                else if (density <= densityMin)
                {
                    speedField[index(i, j)].value[dir] = Particle::MAX_SPEED;
                }
                else
                {
                    //medium speed
                    speedField[index(i, j)].value[dir] = Particle::MAX_SPEED - (density - densityMin) / (densityMax - densityMin) * (4 - flowSpeed);
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
    visited.resize(this->map->getWidth() * this->map->getHeight());
    
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
    flowFlied.resize(this->map->getWidth() * this->map->getHeight());
    
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
    this->avgVelocityField[i] += vec * weight;
}

#pragma mark - Force
b2Vec2 SimulationController::forceFromFlowField(RVOAgent *agent)
{
    b2Vec2 floor = B2Vec2DHelper::floorV(agent->getPosition());
    
    int x = floor.x;
    int y = floor.y;
    
    b2Vec2 f00 = this->map->isInMap(x, y) ? this->flowFlied[index(x, y)] : b2Vec2_zero;
    b2Vec2 f01 = this->map->isInMap(x, y + 1) ? this->flowFlied[index(x, y + 1)] : b2Vec2_zero;
    b2Vec2 f10 = this->map->isInMap(x + 1, y) ? this->flowFlied[index(x + 1, y)] : b2Vec2_zero;
    b2Vec2 f11 = this->map->isInMap(x + 1, y + 1) ? this->flowFlied[index(x + 1, y + 1)] : b2Vec2_zero;
    
    // 水平方向插值
    float32 xWeight = agent->getPosition().x - floor.x;
    
    b2Vec2 top = f00 * (1 - xWeight) +  f10 * xWeight;
    b2Vec2 bottom = f01 * (1 - xWeight) + f11 * xWeight;
    
    // 竖直方向插值
    float32 yWeight = agent->getPosition().y - floor.y;
    
    // 基于水平和竖直方向上的插值，计算得出新的运动方向
    b2Vec2 desiredDirection = top * (1 - yWeight) + bottom * yWeight;
    desiredDirection.Normalize();
    
    // 处于网格中心就不会产生流体力
    if (isnan(desiredDirection.LengthSquared())) {
        return b2Vec2_zero;
    }
    
    return agent->steeringForce(desiredDirection);
}

b2Vec2 SimulationController::forceFromSeparation(RVOAgent *agent)
{
    b2Vec2 totalForce = b2Vec2_zero;
    
    int neighboursCount = 0;
    
    for (int i = 0; i < agents.size(); i++) {
        RVOAgent* other = agents[i];
        if (other != agent) {
            float32 distance = B2Vec2DHelper::distanceTo(other->getPosition(), agent->getPosition());
            if (distance < Particle::MIN_SPEARATION && distance > 0) {
                b2Vec2 pushForce = agent->getPosition() - other->getPosition();
                float32 length = pushForce.Normalize(); //Normalize returns the original length
                float32 r = agent->radius + other->radius;

                totalForce += pushForce * (1 - ((length - r) / (Particle::MIN_SPEARATION - r)));//agent.minSeparation)));
                neighboursCount++;
            }
        }
    }

    if (neighboursCount == 0) {
        return totalForce; //Zero
    }

    return totalForce * (Particle::MAX_FORCE / neighboursCount);
}

b2Vec2 SimulationController::forceFromCohesion(RVOAgent *agent)
{
    //agent.position().Copy();
    b2Vec2 centerOfMass = b2Vec2_zero;
    int neighboursCount = 0;

    for (int i = 0; i < agents.size(); i++)
    {
        RVOAgent *other = agents[i];
        if (other != agent && other->group == agent->group)
        {
            float32 distance = B2Vec2DHelper::distanceTo(agent->getPosition(), other->getPosition());
            if (distance < Particle::MAX_COHESION)
            {
                //sum up the position of our neighbours
                centerOfMass += other->getPosition();
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
    return forceFromSeek(agent, centerOfMass);
}

b2Vec2 SimulationController::forceFromAlignment(RVOAgent *agent)
{
    b2Vec2 averageHeading = b2Vec2_zero;
    int neighboursCount = 0;
    
    //for each of our neighbours (including ourself)
    for (int i = 0; i < agents.size(); i++) {
        RVOAgent *other = agents[i];
        float32 distance = B2Vec2DHelper::distanceTo(agent->getPosition(), other->getPosition());
        //That are within the max distance and are moving
        if (distance < Particle::MAX_COHESION && other->getVelocity().Length() > 0 && agent->group == other->group) {
            //Sum up our headings
            b2Vec2 head = other->getVelocity();
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
    return agent->steeringForce(averageHeading);
}

b2Vec2 SimulationController::forceFromSeek(RVOAgent *agent, b2Vec2& dest)
{
    if (dest.x == agent->getPosition().x && dest.y == agent->getPosition().y) {
        return b2Vec2_zero;
    }
    
    b2Vec2 desired = dest - agent->getPosition();
    
    desired *= (Particle::MAX_SPEED / desired.Length());

    b2Vec2 velocityChange = desired - agent->getVelocity();
    return velocityChange * (Particle::MAX_FORCE / Particle::MAX_SPEED);
}

#pragma mark - Helper
inline int SimulationController::index(int x, int y)const
{
    return x * this->map->getWidth() + y;
}