//
//  SimulationController.h
//  Crowd Simulation
//
//  Created by z on 15/12/16.
//  Copyright © 2015年 SatanWoo. All rights reserved.
//

#ifndef SimulationController_h
#define SimulationController_h

#include "RVO.h"
#include "VirtualNode.h"
#include <unordered_map>
#include <vector>

using namespace std;
using namespace RVO;
typedef enum {
    Normal = 0,
    VirtualNode = 1
} SimulationMode;

class MapController;
class b2Vec2;
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
    const RVOAgent* getAgent(size_t ID)const{return this->agents[ID];}
    
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
    b2Vec2 forceFromFlowField(const Cluster *cluster);
    //b2Vec2 forceFromFlowField(const RVOAgent* agent);
    
    b2Vec2 forceFromSeek(const Cluster *cluster, b2Vec2 dest);
    //b2Vec2 forceFromSeek(const RVOAgent *agent);
    
    b2Vec2 forceFromSeparation(const Cluster *cluster);
    //b2Vec2 forceFromSeparation(const RVOAgent* agent);
    
    b2Vec2 forceFromAlignment(const Cluster *cluster);
    //b2Vec2 forceFromAlignment(const RVOAgent *agent);
    
    b2Vec2 forceFromCohesion(const Cluster *cluster);
    //b2Vec2 forceFromCohesion(const RVOAgent *agent);
    
// Utility
private:
    
// Controller
private:
    MapController *map;
    int index(int x, int y)const;
// Data
private:
    vector<const RVOAgent *> agents;
    
    typedef unordered_map<int, bool> AvaibleAgents;
    AvaibleAgents unclusteredAgents;
    
    vector<const Cluster *> clusters;
    
    size_t count;
    SimulationMode mode;
    
    double timeStep;
    
// Field
private:
    vector<b2Vec2> flowFlied;
    vector<b2Vec2> avgVelocityField;
    
    vector<float> potentialField;
    vector<float> densityField;
    vector<float> discomfortField;
    
    vector<FourGrid> speedField;
    vector<FourGrid> unitCostField;
};

#endif /* SimulationController_h */
