#ifndef _WZTERRAINMAP_H
#define _WZTERRAINMAP_H

#include "MathHelper.h"
#include "Person.h"
#include <iostream>
#include <vector>
#include "Vector.h"

//typedef void (*callback)(Person *p);

class Terrain;
class MapController
{
private:
	Terrain **terrain;
	Person *people;
    MathHelper *helper;

	int m_iWidth;
	int m_iHeight;
	int m_iCount;
    
    Vector2D destinationPoint;

	double m_dTimeStep;
    
protected:
    bool isInMap(int x, int y);
    
    Vector2D seek(int pID, Vector2D des);
    Vector2D separation(int pID, int nID, int &);
    Vector2D cohesion(int pID, int nID, int &);
    Vector2D alignment(int pID, int nID, int &);
    
public:
	MapController(int width, int height, int count, double timeStep = 0.02);
	~MapController();

	void render();
	void update();
	double getTimeStep()const{return m_dTimeStep;}

   
    Vector2D flock(int pID);
  
	std::vector<int> findNeighbours(int pID);

	double density(int neighbourID, int pID);
	Vector2D lamda(int neighbourID, int pID);
	Vector2D deltaP(int neighbourID, int pID);
    void collision(int pID);
    
	bool filterNeightbours(int neighborID, int pID);
	void movePerson(Vector2D old, Vector2D cur, int pID);

	static const double MapGridSize;
	static const double restDensity;
};

#endif