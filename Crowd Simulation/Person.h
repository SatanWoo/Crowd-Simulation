#ifndef _WZPERSON_H
#define _WZPERSON_H

#include "Vector.h"
#include "Box2D.h"
#include <vector>

class MapController;
typedef double (MapController::*calculatorD)(int, int);
typedef b2Vec2 (MapController::*calculatorV)(int, int);
typedef void (MapController::*collision)(int);

typedef b2Vec2 (MapController::*steering)(int);
typedef b2Vec2 (MapController::*flock)(int);

class Person {
public:
	Person();
    Person(MapController *map);
	Person(b2Vec2 pos, b2Vec2 vel, double mass);
	~Person();
    
    // Continuum Crowd
    void steer();
    void flock(flock f);
    
    b2Vec2 getLinearVelocity()const{return body->GetLinearVelocity();}
    b2Vec2 getPosition()const{return body->GetPosition();}

    //
	void init(int pID, b2Vec2 pos = b2Vec2_zero, b2Vec2 vel = b2Vec2_zero, double mass = 1.0);
	void applyAndPredict();
	void updateNeighbours();
	void computeConstraint(calculatorD d);
	void computeLambda(calculatorV l);
	void computeDeltaP(calculatorV l, collision c);

	void setMap(MapController *map){m_mMap = map;}

	b2Vec2 getPos()const{return m_vPos;}
	void setPos(const Vector2D &newPos);

	b2Vec2 getTmpPos()const {return m_vPosTmp;}
	void setTmpPos(const Vector2D &newTmpPos);

	b2Vec2 getVelocity()const {return m_vVelocity;}
	void setVelocity(const Vector2D &newVel);
    
	b2Vec2 getDeltaP()const{return m_deltaP;}
    
    double getMaxSpeed()const{return m_dMaxSpeed;}
    double getMaxForce()const{return m_dMaxForce;}
    double getRadius()const{return m_dRadius;}
    double getMaxCohesion()const{return m_cohesion;}
	
	double getLambda()const {return m_dLambda;}
	double getMass()const{return m_dMass;}
    double getMinSeparation()const{return m_minSeparation;}
    
    void render();
    
protected:
    void initFixtureDef();
    void initBodyDef();
	
private:
    b2FixtureDef *fixtureDef;
    b2Fixture *fixture;
    
    b2Body *body;
    b2BodyDef *bodyDef;
    
	b2Vec2 m_vPos;
	b2Vec2 m_vPosTmp;
	b2Vec2 m_vVelocity;
	b2Vec2 m_deltaP;
    b2Vec2 m_vForce;
    
    double m_dMaxSpeed;
    double m_dMaxForce;
    double m_dRadius;
    double m_minSeparation;
    double m_cohesion;

	double m_dLambda;
	double m_dMass;

	double m_dConstraint;

	MapController *m_mMap;

	int m_iID;

	std::vector<int> neighboursIndex;
};

#endif