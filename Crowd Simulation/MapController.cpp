#include "MapController.h"
#include "Terrain.h"
#include <GLUT/GLUT.h>


const double MapController::restDensity = 1.0;
const double MapController::MapGridSize = 1.5;

MapController::MapController(int width, int height, int count, double timeStep)
{
	m_iWidth = width;
	m_iHeight = height;
	m_iCount = count;
	m_dTimeStep = timeStep;

	people = new Person[count];
   
    double gap = 0.9 * MapGridSize;
    double xPos = gap;
    double yPos = gap;
    
	for (int i = 0; i < count; i++)
	{
        people[i].init(i, Vector2D(xPos, yPos));
        yPos += gap;
        if(yPos >= height - gap){
            yPos = gap;
            xPos += gap;
        }
		people[i].setMap(this);
	}

	terrain = new Terrain *[width];
	for (int i = 0; i < width; i++)
	{
		terrain[i] = new Terrain [height];
	}
    
    for (int i = 0; i < width; i++)
    {
        for (int j = 0; j < height; j++)
        {
            terrain[i][j].setMap(this);
        }
    }

	helper = new MathHelper(1.5);
}

MapController::~MapController()
{
	for (int i = 0; i < m_iWidth; i++)
	{
		delete [] terrain[i];
		terrain[i] = NULL;
	}

	delete [] terrain;
	terrain = NULL;

	delete [] people;
	people = NULL;

	delete helper;
	helper = NULL;
}

void MapController::render()
{    
    glPointSize(2);
    glBegin(GL_POINTS);
    
    glColor3f(1.0, 1.0, 1.0);
    
    for (int i = 0; i < m_iCount; i++) {
        Person &p = people[i];
        p.render();
    }
    
    glEnd();
}

void MapController::update()
{
	for (int i = 0; i < m_iCount; i++)
	{
		Person &p = people[i];
		p.applyAndPredict();
	}

	for (int i = 0; i < m_iCount; i++)
	{
		Person &p = people[i];
		p.updateNeighbours();
	}

	for (int i = 0; i < m_iCount; i++)
	{
		Person &p = people[i];
		p.computeConstraint(&MapController::density);
	}
//
//	for (int i = 0; i < m_iCount; i++)
//	{
//		Person &p = people[i];
//		p.computeLambda(&MapController::lamda);
//	}
//
//	for (int i = 0; i < m_iCount; i++)
//	{
//		Person &p = people[i];
//		p.computeDeltaP(&MapController::deltaP);
//	}
//
//	for (int i = 0; i < m_iWidth; i++)
//	{
//		Person &p = people[i];
//		p.setTmpPos(p.getTmpPos() + p.getDeltaP());
//	}
//
//	for (int i = 0; i < m_iCount; i++)
//	{
//		Person &p = people[i];
//		p.setVelocity((p.getTmpPos() - p.getPos()/m_dTimeStep));
//	}
}

void MapController::movePerson(Vector2D old, Vector2D cur, int pID)
{
	int oldX = old.getX() / MapGridSize;
	int oldY = old.getY() / MapGridSize;

	terrain[oldX][oldY].removePerson(pID);

	int curX = cur.getX() / MapGridSize;
	int curY = cur.getY() / MapGridSize;
	terrain[curX][curY].addPerson(pID);
}

std::vector<int> MapController::findNeighbours(int pID)
{
	static int direction[9][2] = 
	{
		{-1,1}, {-1, 0}, {-1, -1}, 
		{0, 1}, {0, 0}, {0, -1}, 
		{1, 1}, {1, 0}, {1, -1}
	};
    
	Person &p = people[pID];
	int cellX = p.getTmpPos().getX() / MapGridSize;
	int cellY = p.getTmpPos().getY() / MapGridSize;
    
    //cout << "PID" << pID << ":" << cellX << " " << cellY << endl;
    
	std::vector<int> result;
	for (int i = 0; i < 9; i++)
	{
		int curX = cellX + direction[i][0];
        int curY = cellY + direction[i][1];
        
        if (!isInMap(curX, curY)) continue;
        
		Terrain &curUnit = terrain[curX][curY];

		std::vector<int> temp = curUnit.filterPeople(&MapController::filterNeightbours, pID);
        result.insert(result.end(), temp.begin(), temp.end());
	}

	return result;
}

bool MapController::filterNeightbours(int neighborID, int pID)
{
	Person &n = people[neighborID];
	Person &p = people[pID];
	double distance = (p.getTmpPos() - n.getTmpPos()).squaredLength();
    
	return distance < MapGridSize * MapGridSize;
}

double MapController::density(int neighbourID, int pID)
{
	Person &pj = people[neighbourID];
	Person &pi = people[pID];

	return pj.getMass() * helper->poly6(pi.getTmpPos() - pj.getTmpPos());
}

Vector2D MapController::lamda(int neighbourID, int pID)
{
	Person &pj = people[neighbourID];
	Person &pi = people[pID];

	return helper->spikyGrad(pi.getTmpPos() - pj.getTmpPos());
}

Vector2D MapController::deltaP(int neighbourID, int pID)
{
	Person &pj = people[neighbourID];
	Person &pi = people[pID];

	Vector2D result =  helper->spikyGrad(pi.getTmpPos() - pj.getTmpPos()) * (pi.getLambda() + pj.getLambda());

	static double BEDDING = 0.5 * MapGridSize;
	static double REBOUND = -0.5;

	if (pi.getTmpPos().getX() < 0.0 + BEDDING) 
	{
		pi.setTmpPos(Vector2D(BEDDING, pi.getTmpPos().getY()));
		if (pi.getVelocity().getX() < 0.0) 
		{
			pi.setVelocity(Vector2D(pi.getVelocity().getX() * REBOUND, pi.getVelocity().getY()));
		}
	}

	if (pi.getTmpPos().getY() < 0.0 + BEDDING) 
	{
		pi.setTmpPos(Vector2D(pi.getTmpPos().getX(), BEDDING));
		if (pi.getVelocity().getY() < 0.0) 
		{
			pi.setVelocity(Vector2D(pi.getVelocity().getX(), pi.getVelocity().getY() * REBOUND));
		}
	}

	if (pi.getTmpPos().getX() < m_iWidth * MapGridSize - BEDDING) 
	{
		pi.setTmpPos(Vector2D(m_iWidth * MapGridSize - BEDDING, pi.getTmpPos().getY()));
		if (pi.getVelocity().getX() > 0.0) 
		{
			pi.setVelocity(Vector2D(pi.getVelocity().getX() * REBOUND, pi.getVelocity().getY()));
		}
	}

	if (pi.getTmpPos().getY() < m_iHeight * MapGridSize - BEDDING) 
	{
		pi.setTmpPos(Vector2D(pi.getTmpPos().getX(), m_iHeight * MapGridSize - BEDDING));
		if (pi.getVelocity().getY() > 0.0) 
		{
			pi.setVelocity(Vector2D(pi.getVelocity().getX(), pi.getVelocity().getY() * REBOUND));
		}
	}

	return result;
}

bool MapController::isInMap(int x, int y)
{
    if (x < 0 || x >= m_iWidth) return false;
    if (y < 0 || y >= m_iHeight) return false;
    return true;
}
