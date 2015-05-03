#ifndef _WZTERRAINMAP_H
#define _WZTERRAINMAP_H

#include "MathHelper.h"
#include "Person.h"
#include <iostream>
#include <vector>
#include "Vector.h"
#include "Box2D.h"

//typedef void (*callback)(Person *p);

class Terrain;
class MapController
{
private:
	Terrain **terrain;
	Person *people;
    Vector2D **flow;
    MathHelper *helper;
    
    b2World *world;

	int m_iWidth;
	int m_iHeight;
	int m_iCount;
    
    b2Vec2 destinationPoint;

	double m_dTimeStep;
    
protected:
    bool isInMap(int x, int y);
    bool isAccessible(int x, int y);
    
    b2Vec2 steeringFromFlowFleid(int pID, b2Vec2 des);
    b2Vec2 steeringFromLowestCost(int pID, b2Vec2 des);
    b2Vec2 steeringTowards(int pID, b2Vec2 desiredDirection);
    
    b2Vec2 seek(int pID, b2Vec2 des);
    b2Vec2 separation(int pID, int nID, int &);
    b2Vec2 cohesion(int pID, int nID, int &);
    b2Vec2 alignment(int pID, int nID, int &);
    
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
  
	std::vector<int> findNeighbours(int pID);

	double density(int neighbourID, int pID);
	b2Vec2 lamda(int neighbourID, int pID);
	b2Vec2 deltaP(int neighbourID, int pID);
    void collision(int pID);
    
	bool filterNeightbours(int neighborID, int pID);
	void movePerson(b2Vec2 old, b2Vec2 cur, int pID);

	static const double MapGridSize;
	static const double restDensity;
};

#endif