#ifndef _WZTERRAINMAP_H
#define _WZTERRAINMAP_H

#include "Agent.h"
#include "Box2D.h"
#include "CostNode.h"

#include <iostream>
#include <vector>
#include "Definition.h"

#include "RVOTree.h"

class VirtualNode;

typedef enum {
  Normal = 0,
  Virtual = 1
} MapControllerState;

class MapController
{
private:
    b2Vec2 **flow;
    b2Vec2 **avgVelocityField;
    
    float32 **potentialField;
    float32 **densityField;
    float32 **discomfortField;
    FourGrid **speedField;
    FourGrid **costField;
    
    bool **visited;
    
    b2Vec2** initializeVecField();
    float32** initializeFloatField();
    
    void deinitializeField(b2Vec2 **field);
    void deinitializeField(float32 **field);
    
private:
    std::vector<Agent *> agents;
    std::vector<VirtualNode *> nodes;
    std::vector<Agent *> leadingAgents;
    
    std::vector<b2Vec2> destinationPoints;
    std::vector<b2Vec2> obstacles;
    
    HashMap availableAgents;
    
    RVOTree *tree;
    b2World *world;
    
    double m_dTimeStep;
    
    size_t IDcounter;
    size_t groupCounter;
    
	int m_iWidth;
    int m_iHeight;
    
    MapControllerState simulationMode;
    
private:
    void buildKDTree();
    void computerNearestNeighbours();
    void mergeNode();
    
private:
    void updateContinuumCrowdData();
    void ccClearBuffers();
    void ccCalculateDensityAndAverageSpeed();
    void ccCalculateUnitCostField();
    
    void ccAddDensity(int x, int y, const b2Vec2& vec, float32 weight);
    void ccGenerateFlowField();
    void ccClearPotentialField();
    
    void ccPotentialFieldEikonalFill(b2Vec2 des);

private:
    b2Vec2 steeringBehaviourFlowField(Agent *agent);
    b2Vec2 steeringBehaviourSeek(Agent *agent, b2Vec2 dest);
    b2Vec2 steeringBehaviourSeparation(Agent *agent);
    b2Vec2 steeringBehaviourAlignment(Agent *agent);
    b2Vec2 steeringBehaviourCohesion(Agent *agent);
    b2Vec2 steerTowards(Agent *agent, b2Vec2 direction);
    
private:
    void renderBackground();
    void renderAgents();
    void renderObstacels();
    
    bool isValid(int x, int y);
    
    void simulateAgents(const std::vector<Agent *> agents);
    
public:
	MapController(int width, int height, int count, double timeStep = 0.02);
	~MapController();
    
    void updateDestinationPoint(b2Vec2 newDest);
    void switchState();

    b2World* getWorld()const{return world;}
    
	void render();
	void update();
    
	double getTimeStep()const{return m_dTimeStep;}
    b2Vec2 flock(int pID);

	static const double MapGridSize;
	static const double restDensity;
};

#endif