#include "Terrain.h"
#include <iterator>
#include <GLUT/GLUT.h>
#include "MapController.h"

Terrain::Terrain(double x /* = 0.0 */, double y /* = 0.0 */, double z /* = 0.0 */)
{
	m_dX = x;
	m_dY = y;
	m_dZ = z;
    
    obstacle = -1;
    m_iDistance = -1;
}

Terrain::~Terrain()
{
	people.clear();
    m_map = NULL;
}

void Terrain::init(int i)
{
    obstacle = i;
}

void Terrain::addPerson(int index)
{
	people.insert(index);
}

void Terrain::removePerson(int index)
{
    people.erase(index);
}

void Terrain::render()
{
}

std::vector<int> Terrain::filterPeople(filter f, int pID)
{
	std::set<int>::iterator it;
	std::vector<int> result;

	for (it = people.begin(); it != people.end(); ++it)
	{
        if ((m_map->*f)(*it, pID)) {
            result.push_back(*it);
        }
	}
    
	return result;
}