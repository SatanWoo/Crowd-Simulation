#include "Person.h"
#include "MapController.h"
#include <GLUT/GLUT.h>

Person::Person()
{
	init(0, b2Vec2_zero, b2Vec2_zero, 1.0);
}

Person::Person(MapController *map)
{
    setMap(map);
    init(0, b2Vec2_zero, b2Vec2_zero, 1.0);
}

Person::Person(b2Vec2 pos, b2Vec2 vel, double mass)
{
	init(0, pos, vel, mass);
}

Person::~Person()
{
    delete fixtureDef->shape;
    fixtureDef->shape = NULL;
    
    delete fixtureDef;
    fixtureDef = NULL;
    
    delete bodyDef;
    bodyDef = NULL;
    
	neighboursIndex.clear();
    m_mMap = NULL;
}

void Person::initBodyDef()
{
    bodyDef = new b2BodyDef();
    bodyDef->type = b2_dynamicBody;
    bodyDef->position.Set(m_vPos.x, m_vPos.y);
    
    body = m_mMap->getWorld()->CreateBody(bodyDef);
    initFixtureDef();
}

void Person::initFixtureDef()
{
    fixtureDef = new b2FixtureDef();
    fixtureDef->density = 10.0;
    fixtureDef->friction = 0.5;
    fixtureDef->restitution = 0.2;
    fixtureDef->shape = new b2CircleShape();
    
    fixture = body->CreateFixture(fixtureDef);
}

void Person::init(int pID, b2Vec2 pos, b2Vec2 vel, double mass)
{
    m_dMaxForce = 20;
    m_dMaxSpeed = 4;
    m_dRadius = 0.4;
    m_minSeparation = 0.8;
    m_cohesion = 2;
    
	m_iID = pID;
	m_vPos = pos;
	m_vVelocity = vel;
	m_vPosTmp = b2Vec2_zero;
	m_dMass = mass;
}

void Person::steer()
{
    b2Vec2 force = m_vForce;
    m_vVelocity += force * m_mMap->getTimeStep();
    double curSpeed = m_vVelocity.Length();
    if (curSpeed > m_dMaxSpeed) {
        m_vVelocity *= m_dMaxSpeed / curSpeed;
    }
    
    this->body->ApplyLinearImpulse(m_vVelocity, m_vPos);
    m_vPos += m_vVelocity * m_mMap->getTimeStep();
}

void Person::flock(::flock f)
{
    m_vForce = (m_mMap->*f)(m_iID);
}

void Person::setPos(const Vector2D &newPos)
{
    m_vPos.x = newPos.getX();
    m_vPos.y = newPos.getY();
}

void Person::setTmpPos(const Vector2D &newTmpPos)
{
    m_vPosTmp.x = newTmpPos.getX();
    m_vPosTmp.y = newTmpPos.getY();
}

void Person::setVelocity(const Vector2D &newVel)
{
//    m_vVelocity.x = newVel.getX();
//    m_vVelocity.y = newVel.getY();
}

void Person::applyAndPredict()
{
	neighboursIndex.clear();
    
//    m_vVelocity = m_vVelocity + Vector2D(0.0, -9.8) / m_dMass * m_mMap->getTimeStep();
//
//	Vector2D old = m_vPosTmp;
//    m_vPosTmp = m_vPos + m_vVelocity * m_mMap->getTimeStep();
//	m_mMap->movePerson(old, m_vPosTmp, m_iID);
}

void Person::updateNeighbours()
{
//	neighboursIndex.clear();
//    
//	std::vector<int> neighbours = m_mMap->findNeighbours(m_iID);
//	neighboursIndex.assign(neighbours.begin(), neighbours.end());
}

void Person::render()
{
//    glColor3f(1.0, (m_dConstraint + 1) * MapController::restDensity / 2.0, 0.5);
    glVertex2f(m_vPos.x, m_vPos.y);
}

void Person::computeLambda(calculatorV d)
{
//	size_t size = neighboursIndex.size();
//	double total = 0.0f;
//	for (int i = 0; i < size; i++)
//	{
//		int neighbourID = neighboursIndex[i];
//		Vector2D temp = Vector2D::vec2Zero;
//		if (neighbourID == m_iID) 
//		{
//			for (int j = 0; j < size; j++)
//			{
//				int curID = neighboursIndex[j];
//				if (curID == m_iID) continue;
//
//				temp += (m_mMap->*d)(curID, m_iID);
//			}
//		}
//		else
//		{
//			temp = (m_mMap->*d)(neighbourID, m_iID) * (-1.0);
//		}
//		total += (temp / MapController::restDensity).length();
//	}
//
//	m_dLambda = m_dConstraint * (-1.0) / (total + 50.0);
}

void Person::computeDeltaP(calculatorV l, collision c)
{
//	size_t size = neighboursIndex.size();
//	Vector2D deltaP = Vector2D(0, 0);
//    
//	for (int i = 0; i < size; i++)
//	{
//		int nID = neighboursIndex[i];
//        
//        Vector2D temp = (m_mMap->*l)(nID, m_iID);
//		deltaP += temp;
//	}
//    
//	m_deltaP = deltaP / MapController::restDensity;
//    (m_mMap->*c)(m_iID);
}

void Person::computeConstraint(calculatorD d)
{
//	size_t size = neighboursIndex.size();
//	double total = 0.0f;
//	for (int i = 0; i < size; i++)
//	{
//		int neighbourID = neighboursIndex[i];
//		total += (m_mMap->*d)(neighbourID, m_iID);
//	}
//
//    m_dConstraint = total / MapController::restDensity - 1;
}