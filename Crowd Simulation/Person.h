#ifndef _WZPERSON_H
#define _WZPERSON_H

#include "Vector.h"
#include <vector>

class MapController;
typedef double (MapController::*calculatorD)(int, int);
typedef Vector2D (MapController::*calculatorV)(int, int);
typedef void (MapController::*collision)(int);

typedef Vector2D (MapController::*steering)(int);
typedef Vector2D (MapController::*flock)(int);

class Person {
public:
	Person();
	Person(Vector2D pos, Vector2D vel, double mass);
	~Person();
    
    // Steer
    void steer();
    void flock(flock f);

	void init(int pID, Vector2D pos = Vector2D::vec2Zero, Vector2D vel = Vector2D::vec2Zero, double mass = 1.0);
	void applyAndPredict();
	void updateNeighbours();
	void computeConstraint(calculatorD d);
	void computeLambda(calculatorV l);
	void computeDeltaP(calculatorV l, collision c);

	void setMap(MapController *map){m_mMap = map;}

	Vector2D getPos()const{return m_vPos;}
	void setPos(const Vector2D &newPos);

	Vector2D getTmpPos()const {return m_vPosTmp;}
	void setTmpPos(const Vector2D &newTmpPos);

	Vector2D getVelocity()const {return m_vVelocity;}
	void setVelocity(const Vector2D &newVel);
	
	Vector2D getDeltaP()const{return m_deltaP;}
    
    double getMaxSpeed()const{return m_dMaxSpeed;}
    double getMaxForce()const{return m_dMaxForce;}
    double getRadius()const{return m_dRadius;}
	
	double getLambda()const {return m_dLambda;}
	double getMass()const{return m_dMass;}
    
    void render();
	
private:
	Vector2D m_vPos;
	Vector2D m_vPosTmp;
	Vector2D m_vVelocity;
	Vector2D m_deltaP;
    Vector2D m_vForce;
    
    double m_dMaxSpeed;
    double m_dMaxForce;
    double m_dRadius;

	double m_dLambda;
	double m_dMass;

	double m_dConstraint;

	MapController *m_mMap;

	int m_iID;

	std::vector<int> neighboursIndex;
};

#endif