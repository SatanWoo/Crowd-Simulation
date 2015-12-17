//
//  SimulationController.h
//  Crowd Simulation
//
//  Created by z on 15/12/16.
//  Copyright © 2015年 SatanWoo. All rights reserved.
//

#ifndef SimulationController_h
#define SimulationController_h

#include "Particle.h"
#include "RVO.h"
//#include "VirtualNode.h"
#include "Box2D.h"
#include <unordered_map>
#include <vector>

using namespace std;
using namespace RVO;
typedef enum {
    Normal = 0,
    VirtualNode = 1
} SimulationMode;

class MapController;
class FourGrid;

typedef struct VirtualNode Cluster;

class SimulationController
{
public:
    SimulationController(size_t count);
    ~SimulationController();
    
    void init(int w, int h);
    void simulate();
    void stop();
    void enableVirtualNodeMode();
    
    void render();
    
    void setTimeStep(double time){this->timeStep = time;}
    double getTimeStep()const{return this->timeStep;}
    
    RVOAgent* getAgent(size_t ID)const{return this->agents[ID];}
    
    void setRelation(b2Vec3 relation){this->relation = relation;}
    b2Vec3 getRelation()const{return this->relation;}
    
protected:
    void continuumSimulation();
    void flockSimulation();
    void clusterSimulation();
    
// Continnum Step
private:
    void resetContinnum();
    
    //Update density field and average speed map (4.1)
    void calculateDensityAndAverageSpeed();
    
    //CC Paper says this is group dependant, but I'm not sure how...
    void calculateUnitCostField();
    
    void buildDensityField(int x, int y, const b2Vec2& vec, float weight);
    
    void buildPotentialField(const b2Vec2& goal);
    void buildFlowField();
    
private:
    //b2Vec2 forceFromFlowField(Particle *p);
    //b2Vec2 forceFromFlowField(Cluster *cluster);
    b2Vec2 forceFromFlowField(RVOAgent* agent);
    
    //b2Vec2 forceFromSeek(Particle *p);
    //b2Vec2 forceFromSeek(Cluster *cluster, b2Vec2& dest);
    b2Vec2 forceFromSeek(RVOAgent *agent, b2Vec2& dest);
    
    //b2Vec2 forceFromSeparation(Particle *p);
    //b2Vec2 forceFromSeparation(Cluster *cluster);
    b2Vec2 forceFromSeparation(RVOAgent* agent);
    
    //b2Vec2 forceFromAlignment(Particle *p);
    //b2Vec2 forceFromAlignment(Cluster *cluster);
    b2Vec2 forceFromAlignment(RVOAgent *agent);
    
    //b2Vec2 forceFromCohesion(Particle *p);
    //b2Vec2 forceFromCohesion(Cluster *cluster);
    b2Vec2 forceFromCohesion(RVOAgent *agent);
    
private:
    
    
// Utility
private:
    int index(int x, int y)const;
    
// Controller
private:
    MapController *map;
    b2World *world;
    
    b2Vec3 relation;
    double timeStep;
    
    size_t count;
    SimulationMode mode;
    
// Data
private:
    vector<RVOAgent *> agents;
    vector<b2Vec2 *> obstacles;
    
    RVOAgent::AgentsPool unclusteredAgents;
    
    vector<Cluster *> clusters;
    
    RVOKDTree *tree;

    
// Field
private:
    vector<b2Vec2> flowFlied;
    vector<b2Vec2> avgVelocityField;
    
    vector<float> obstacleField;
    
    vector<float> potentialField;
    vector<float> densityField;
    vector<float> discomfortField;
    
    vector<FourGrid> speedField;
    vector<FourGrid> unitCostField;
};

#endif /* SimulationController_h */