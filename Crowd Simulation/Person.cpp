#include "Person.h"
#include "MapController.h"
#include <GLUT/GLUT.h>

Person::Person()
{
	init(0, Vector2D::vec2Zero, Vector2D::vec2Zero, 1.0);
}

Person::Person(Vector2D pos, Vector2D vel, double mass)
{
	init(0, pos, vel, mass);
}

Person::~Person()
{
	neighboursIndex.clear();
}

void Person::init(int pID, Vector2D pos, Vector2D vel, double mass)
{
	m_iID = pID;
	m_vPos = pos;
	m_vVelocity = vel;
	m_vPosTmp = Vector2D::vec2Zero;
	m_dMass = mass;
}

void Person::setPos(const Vector2D &newPos)
{
	m_vPos.setX(newPos.getX());
	m_vPos.setY(newPos.getY());
}

void Person::setTmpPos(const Vector2D &newTmpPos)
{
	m_vPosTmp.setX(newTmpPos.getX());
	m_vPosTmp.setY(newTmpPos.getY());
}

void Person::setVelocity(const Vector2D &newVel)
{
	m_vVelocity.setX(newVel.getX());
	m_vVelocity.setY(newVel.getY());
}

void Person::applyAndPredict()
{
	neighboursIndex.clear();
    
    m_vVelocity = m_vVelocity + Vector2D(0.0, -9.8) / m_dMass * m_mMap->getTimeStep();

	Vector2D old = m_vPosTmp;
    m_vPosTmp = m_vPos + m_vVelocity * m_mMap->getTimeStep();
	m_mMap->movePerson(old, m_vPosTmp, m_iID);
}

void Person::updateNeighbours()
{
	neighboursIndex.clear();
    
	std::vector<int> neighbours = m_mMap->findNeighbours(m_iID);
	neighboursIndex.assign(neighbours.begin(), neighbours.end());
}

void Person::render()
{
    glVertex2f(m_vPos.getX(), m_vPos.getY());
}

void Person::computeLambda(calculatorV d)
{
	int size = neighboursIndex.size();
	double total = 0.0f;
	for (int i = 0; i < size; i++)
	{
		int neighbourID = neighboursIndex[i];
		Vector2D temp = Vector2D::vec2Zero;
		if (neighbourID == m_iID) 
		{
			for (int j = 0; j < size; j++)
			{
				int curID = neighboursIndex[j];
				if (curID == m_iID) continue;

				temp += (m_mMap->*d)(curID, m_iID);
			}
		}
		else
		{
			temp = (m_mMap->*d)(neighbourID, m_iID) * (-1.0);
		}
		total += (temp / MapController::restDensity).length();
	}

	m_dLambda = m_dConstraint * (-1) / (total + 50.0);
    std::cout << m_dLambda << " ";
}

void Person::computeDeltaP(calculatorV l)
{
	size_t size = neighboursIndex.size();
	Vector2D deltaP = Vector2D::vec2Zero;
	for (int i = 0; i < size; i++)
	{
		int nID = neighboursIndex[i];
		deltaP = deltaP + (m_mMap->*l)(nID, m_iID);
	}

	m_deltaP = deltaP / MapController::restDensity;
}

void Person::computeConstraint(calculatorD d)
{
	size_t size = neighboursIndex.size();
	double total = 0.0f;
	for (int i = 0; i < size; i++)
	{
		int neighbourID = neighboursIndex[i];
		total += (m_mMap->*d)(neighbourID, m_iID);
	}

    m_dConstraint = total / MapController::restDensity - 1;
    //std::cout << m_dConstraint << " ";
}