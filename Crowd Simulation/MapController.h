#ifndef _WZTERRAINMAP_H
#define _WZTERRAINMAP_H

#include "MathHelper.h"
#include "Person.h"
#include <iostream>
#include <vector>
#include "Box2D.h"

//typedef void (*callback)(Person *p);

class Terrain;
class MapController
{
private:
	Terrain **terrain;
	Person *people;
    b2Vec2 **flow;
    
    b2Vec2 **potentialField;
    b2Vec2 **costField;
    b2Vec2 **speedField;
    b2Vec2 **avgVelocityField;
    
    float32 **densityField;
    float32 **discomfortField;
    
    MathHelper *helper;
    
    b2World *world;

	int m_iWidth;
	int m_iHeight;
	int m_iCount;
    
    b2Vec2 destinationPoint;
    
    //std::vector<b2Vec2> des;

	double m_dTimeStep;
    
protected:
    bool isInMap(int x, int y);
    bool isAccessible(int x, int y);
    
    b2Vec2** initializeVecField();
    float32** initializeFloatField();
    
    void deinitializeField(b2Vec2 **field);
    void deinitializeField(float32 **field);
    
    void updateContinuumCrowdData();
    void ccClearBuffers();
    void ccCalculateDensityAndAverageSpeed();
    void ccCalculateUnitCostField();
    
    void ccAddDensity(int x, int y, const b2Vec2& vec, float32 weight);
    void ccGenerateFlowField();
    void ccClearPotentialField();
    
    void ccPotentialFieldEikonalFill(b2Vec2 des);
    
    b2Vec2 steeringFromFlowFleid(int pID, b2Vec2 des);
    b2Vec2 steeringFromSeek(int pID, b2Vec2 des);
    b2Vec2 steeringFromSeparation(int pID, b2Vec2 des);
    b2Vec2 steeringFromAlignment(int pID, b2Vec2 des);
    b2Vec2 steeringFromCohesion(int pID, b2Vec2 des);
    b2Vec2 steeringFromAvoidance(int pID, b2Vec2 des);
    b2Vec2 steeringFromLowestCost(int pID, b2Vec2 des);
    b2Vec2 steeringTowards(int pID, b2Vec2 desiredDirection);
        
    void buildDijkstra();
    void buildFlowField();
    
    std::vector<b2Vec2> fourAdjacentNeighbours(const b2Vec2& vec);
    std::vector<b2Vec2> eightAdjacentNeighbours(const b2Vec2& vec);
    
public:
	MapController(int width, int height, int count, double timeStep = 0.02);
	~MapController();

    b2World* getWorld()const{return world;}
    
	void render();
	void update();
    void setDestionationPoint(b2Vec2 point){destinationPoint = point;}
	double getTimeStep()const{return m_dTimeStep;}

   
    b2Vec2 flock(int pID);
  
//	std::vector<int> findNeighbours(int pID);
//
//	double density(int neighbourID, int pID);
//	b2Vec2 lamda(int neighbourID, int pID);
//	b2Vec2 deltaP(int neighbourID, int pID);
//    void collision(int pID);
//    
//	bool filterNeightbours(int neighborID, int pID);
//	void movePerson(b2Vec2 old, b2Vec2 cur, int pID);

	static const double MapGridSize;
	static const double restDensity;
};

#endif