#ifndef _WZTERRAINMAP_H
#define _WZTERRAINMAP_H

#include "Box2D.h"
#include "RVO.h"
#include "CostNode.h"

#include <vector>

using namespace std;

class MapController
{
public:
    void setMapSize(int width, int height){this->width = width; this->height = height;}
    int getWidth()const{return this->width;}
    int getHeight()const{return this->height;}
    bool isInMap(int x, int y)const;
    
private:
    int width, height;
    
    RVO::RVOKDTree *tree;
    
    std::vector<const RVO::RVOAgent *> agents;
    std::vector<b2Vec2> destinationPoints;
    std::vector<b2Vec2> obstacles;
    
    b2Vec2 **flow;
    b2Vec2 **avgVelocityField;
    
    float32 **potentialField;
    float32 **densityField;
    float32 **discomfortField;
    FourGrid **speedField;
    FourGrid **costField;
    
    bool **visited;
        
    b2World *world;
    
	int m_iWidth;
	int m_iHeight;
    
	double m_dTimeStep;
    
protected:
    bool isValid(int x, int y);
    
    void buildKDTree();
    void computerNearestNeighbours(double radius);
    void mergeNode();
    
    b2Vec2** initializeVecField();
    float32** initializeFloatField();
    
    void deinitializeField(b2Vec2 **field);
    void deinitializeField(float32 **field);
    
    void updateContinuumCrowdData();
    void ccCalculateDensityAndAverageSpeed();
    void ccCalculateUnitCostField();
    
    void ccAddDensity(int x, int y, const b2Vec2& vec, float32 weight);
    void ccGenerateFlowField();
    void ccClearPotentialField();
    
    void ccPotentialFieldEikonalFill(b2Vec2 des);
    
//    b2Vec2 steeringBehaviourFlowField(VirtualNode &agent);
//    b2Vec2 steeringBehaviourSeek(VirtualNode &agent, b2Vec2 dest);
//    b2Vec2 steeringBehaviourSeparation(VirtualNode &agent);
//    b2Vec2 steeringBehaviourAlignment(VirtualNode &agent);
//    b2Vec2 steeringBehaviourCohesion(VirtualNode &agent);
//    b2Vec2 steerTowards(VirtualNode &agent, b2Vec2 direction);
    
public:
	MapController(int width, int height, int count, double timeStep = 0.02);
	~MapController();
    
    b2Vec2 getPosition(size_t ID)const;
    
    void updateDestinationPoint(b2Vec2 newDest);

    b2World* getWorld()const{return world;}
    
	void render();
	void update();
    
	double getTimeStep()const{return m_dTimeStep;}
    b2Vec2 flock(int pID);

	static const double MapGridSize;
	static const double restDensity;
    
    bool enableVirtualNode;
};

#endif