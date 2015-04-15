#ifndef _WZTERRAINUNIT_H
#define _WZTERRAINUNIT_H

#include "Person.h"
#include <set>

using namespace std;

class MapController;
typedef bool(MapController::*filter)(int, int);

class Terrain {
protected:
	double m_dX; // Top Left
	double m_dY; // Top Left
	double m_dZ;
    
    int m_iDistance;
    int obstacle;
    
	set<int> people;

	MapController *m_map;
public:
	Terrain(double x = 0.0, double y = 0.0, double z = 0.0);

    void init(int i);
    
	void addPerson(int index);
	void removePerson(int index);
    void setMap(MapController *map){m_map = map;}
	vector<int> filterPeople(filter f, int pID);
    
    void setDistance(int dis){m_iDistance = dis;}
    int getDistance()const{return m_iDistance;}

	virtual int obstacleCoefficient()const{return obstacle;}
    virtual void render();
	virtual int maxPeople()const{return INT_MAX;}
	virtual ~Terrain();
};

#endif