#ifndef _WZTERRAINUNIT_H
#define _WZTERRAINUNIT_H

#include "Person.h"
#include <set>

class MapController;
typedef bool(MapController::*filter)(int, int);

class Terrain {
private:
	double m_dX;
	double m_dY;
	double m_dZ;
    
	std::set<int> people;

	MapController *m_map;
public:
	Terrain(double x = 0.0, double y = 0.0, double z = 0.0);

	void addPerson(int index);
	void removePerson(int index);
	void setMap(MapController *map);
	std::vector<int> filterPeople(filter f, int pID);

	virtual int obstacleCoefficient()const{return 0;}
	virtual int maxPeople()const{return INT_MAX;}
	virtual ~Terrain();
};

#endif