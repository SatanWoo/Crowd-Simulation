#ifndef _WZTERRAINMAP_H
#define _WZTERRAINMAP_H

#include "Agent.h"
#include <iostream>
#include <vector>
#include "Box2D.h"
#include "CostNode.h"

class Terrain;
class MapController
{
private:
    
    std::vector<Agent> agents;
    std::vector<b2Vec2> destinationPoints;
    std::vector<b2Vec2> obstacles;
    
    b2Vec2 **flow;
    bool **lost;
    float32 **dijkstra;
    
    b2World *world;

	int m_iWidth;
	int m_iHeight;
    
	double m_dTimeStep;
    
protected:
    bool isValid(int x, int y);
    
    b2Vec2** initializeVecField();
    float32** initializeFloatField();
    
    void deinitializeField(b2Vec2 **field);
    void deinitializeField(float32 **field);
    void deinitializeField(bool **field);
    
    void ccGenerateFlowField();
    void ccGenerateDijkstraField();
    void calculateLost(CostNode at, CostNode pathEnd);
    
    b2Vec2 steeringBehaviourFlowField(Agent &agent);
    b2Vec2 steeringBehaviourSeek(Agent &agent, b2Vec2 dest);
    b2Vec2 steeringBehaviourSeparation(Agent &agent);
    b2Vec2 steeringBehaviourAlignment(Agent &agent);
    b2Vec2 steeringBehaviourCohesion(Agent &agent);
    b2Vec2 steerTowards(Agent &agent, b2Vec2 direction);
    
    std::vector<b2Vec2> allNeighbours(b2Vec2& pos);
    std::vector<CostNode> directNeighbours(CostNode& pos);
    
public:
	MapController(int width, int height, int count, double timeStep = 0.02);
	~MapController();
    
    void updateDestinationPoint(b2Vec2 newDest);

    b2World* getWorld()const{return world;}
    
	void render();
	void update();
    
	double getTimeStep()const{return m_dTimeStep;}

	static const double MapGridSize;
	static const double restDensity;
};

#endif