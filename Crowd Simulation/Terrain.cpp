#include "Terrain.h"
#include <iterator>
#include "MapController.h"

Terrain::Terrain(double x /* = 0.0 */, double y /* = 0.0 */, double z /* = 0.0 */)
{
	m_dX = x;
	m_dY = y;
	m_dZ = z;
    
    people.insert(5);

}

Terrain::~Terrain()
{
	people.clear();
}

void Terrain::addPerson(int index)
{
	people.insert(index);
}

void Terrain::removePerson(int index)
{
    people.erase(index);
//    std::set<int>::iterator it = people.begin();
//    
//    while (it != people.end()) {
//        if (*it++ == index) {
//            
//            break;
//        }
//    }
}

std::vector<int> Terrain::filterPeople(filter f, int pID)
{
	std::set<int>::iterator it;
	std::vector<int> result;
	for (it = people.begin(); it != people.end(); ++it)
	{
		if ((m_map->*f)(*it, pID)) result.push_back(*it);
	}
	return result;
}