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
    
	set<int> people;

	MapController *m_map;
public:
	Terrain(double x = 0.0, double y = 0.0, double z = 0.0);

	void addPerson(int index);
	void removePerson(int index);
    void setMap(MapController *map){m_map = map;}
	vector<int> filterPeople(filter f, int pID);

	virtual int obstacleCoefficient()const{return 0;}
    virtual void render();
	virtual int maxPeople()const{return INT_MAX;}
	virtual ~Terrain();
};

#endif