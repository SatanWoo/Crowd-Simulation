#include "MapController.h"
#include "Terrain.h"
#include <GLUT/GLUT.h>
#include <math.h>


const double MapController::restDensity = 1.0;
const double MapController::MapGridSize = 1.5;

MapController::MapController(int width, int height, int count, double timeStep)
{
	m_iWidth = width;
	m_iHeight = height;
	m_iCount = count;
	m_dTimeStep = timeStep;

	people = new Person[count];
    srand( (unsigned)time(NULL));
    
	for (int i = 0; i < count; i++)
	{
        Vector2D pos = Vector2D(rand() % m_iWidth * MapGridSize, rand()% m_iHeight * MapGridSize);
        people[i].init(i, pos);
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
    
    destinationPoint.setX(rand() % m_iWidth * MapGridSize);
    destinationPoint.setY(rand()% m_iHeight * MapGridSize);
    
    cout << destinationPoint.getX() << " || " << destinationPoint.getY() << endl;

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
    glPointSize(10);
    glBegin(GL_POINTS);
    glColor4f(0.0, 1.0, 0.0, 1.0);
    glVertex2d(destinationPoint.getX(), destinationPoint.getY());
    glEnd();
    
    glLineWidth(1);
    glBegin(GL_LINES);
    
    static int fourDir[4][2] = {{-1, 0}, {1, 0}, {0, -1}, {0, 1}};
    glColor4f(1.0f, 1.0f, 1.0f, 0.1);
    
    for (int i = 0; i < m_iWidth; i++) {
        for (int j = 0; j < m_iHeight; j++) {
            double xPos = i * MapGridSize;
            double yPos = j * MapGridSize;
            
            for (int k = 0; k < 4; k++) {
                int kx = i + fourDir[k][0];
                int ky = j + fourDir[k][1];
                
                if (isInMap(kx, ky)) {
                    glVertex2d(xPos, yPos);
                    glVertex2d(kx * MapGridSize, ky * MapGridSize);
                }
            }
        }
    }
    glEnd();

    glPointSize(5);
    glBegin(GL_POINTS);
    for (int i = 0; i < m_iCount; i++) {
        Person &p = people[i];
        p.render();
    }
    glEnd();
}

Vector2D MapController::seek(int pID, Vector2D des)
{
    Person &pi = people[pID];
    Vector2D distance = des - pi.getPos();
    Vector2D desiredSpeed = distance * (pi.getMaxSpeed() / distance.squaredLength());
    Vector2D velocityChange = desiredSpeed - pi.getVelocity();
    
    return velocityChange * (pi.getMaxForce() / pi.getMaxSpeed());
}

Vector2D MapController::flock(int pID)
{
    Person &pi = people[pID];
    
    Vector2D allForce = Vector2D::vec2Zero;
    
    Vector2D seekForce = seek(pID, destinationPoint);
    Vector2D sepForce = Vector2D::vec2Zero, cohForce = pi.getPos(), alignForce = Vector2D::vec2Zero;
    int sepNeighbour = 1, cohNeighbour = 1, alignNeighbour = 1;
    for (int i = 0; i < m_iCount; i++) {
        if (i == pID) continue;
        
        sepForce += separation(pID, i, sepNeighbour);
        cohForce += cohesion(pID, i, cohNeighbour);
        //alignForce += alignment(pID, i, alignNeighbour);
    }
    
    sepForce /= sepNeighbour;
    sepForce *= pi.getMaxForce();
    
    cohForce /= cohNeighbour;
    cohForce = seek(pID, cohForce);
    cohForce = cohNeighbour == 1 ? Vector2D::vec2Zero : cohForce ;
    
    alignForce /= alignNeighbour;
    alignForce *= pi.getMaxSpeed();
    alignForce -= pi.getVelocity();
    alignForce *= (pi.getMaxForce() / pi.getMaxSpeed());
    
    allForce = seekForce + (sepForce * 2) + (cohForce * 0.2) + alignForce * 0.5;
    
    if (allForce.squaredLength() > pi.getMaxForce())
    {
        allForce = allForce.normalize() * pi.getMaxForce();
    }
    
    return allForce;
}

Vector2D MapController::separation(int pID, int nID, int& count)
{
    Person &pi = people[pID];
    Person &pn = people[nID];
    
    double distance = pi.getPos().distanceTo(pn.getPos());
    
    Vector2D pushForce = Vector2D::vec2Zero;
    
    if (distance > 0 && distance < pi.getRadius())
    {
        count = count + 1;
        Vector2D dis = pi.getPos() - pn.getPos();
        pushForce += dis.normalize() * (1 - dis.squaredLength() / pi.getRadius());
    }

    return pushForce;
}

Vector2D MapController::cohesion(int pID, int nID, int& count)
{
    Person &pi = people[pID];
    Person &pn = people[nID];
    
    double distance = pi.getPos().distanceTo(pn.getPos());
    
    if (distance < pi.getRadius())
    {
        count = count + 1;
        return pn.getPos();
    }
    
    return Vector2D::vec2Zero;
}

Vector2D MapController::alignment(int pID, int nID, int &count)
{
    Person &pi = people[pID];
    Person &pn = people[nID];
    
    double distance = pi.getPos().distanceTo(pn.getPos());
    
    if (distance < pi.getRadius() && pn.getVelocity().squaredLength() > 0)
    {
        count = count + 1;
        return pn.getVelocity().normalize();
    }
    
    return Vector2D::vec2Zero;
}

void MapController::update()
{
    for (int i = 0; i < m_iCount; i++) {
        Person &pi = people[i];
        pi.flock(&MapController::flock);
    }
    
    for (int i = 0; i < m_iCount; i++) {
        Person &pi = people[i];
        pi.steer();
    }
//
//	for (int i = 0; i < m_iCount; i++)
//	{
//		Person &p = people[i];
//		p.applyAndPredict();
//	}
//
//	for (int i = 0; i < m_iCount; i++)
//	{
//		Person &p = people[i];
//		p.updateNeighbours();
//	}
//    
//    static int iteration = 3;
//    
//    for (int w = 0; w < iteration; w++) {
//        for (int i = 0; i < m_iCount; i++)
//        {
//            Person &p = people[i];
//            p.computeConstraint(&MapController::density);
//        }
//        
//        for (int i = 0; i < m_iCount; i++)
//        {
//            Person &p = people[i];
//            p.computeLambda(&MapController::lamda);
//        }
//        
//        for (int i = 0; i < m_iCount; i++)
//        {
//            Person &p = people[i];
//            p.computeDeltaP(&MapController::deltaP, &MapController::collision);
//        }
//        
//        for (int i = 0; i < m_iCount; i++)
//        {
//            Person &p = people[i];
//            p.setTmpPos(p.getTmpPos() + p.getDeltaP());
//        }
//    }
//
//	for (int i = 0; i < m_iCount; i++)
//	{
//		Person &p = people[i];
//		p.setVelocity((p.getTmpPos() - p.getPos())/m_dTimeStep);
//        p.setPos(p.getTmpPos());
//	}
}

void MapController::movePerson(Vector2D old, Vector2D cur, int pID)
{
	int oldX = old.getX() / MapGridSize;
	int oldY = old.getY() / MapGridSize;
    if (isInMap(oldX, oldY))
        terrain[oldX][oldY].removePerson(pID);

	int curX = cur.getX() / MapGridSize;
	int curY = cur.getY() / MapGridSize;
    if (isInMap(curX, curY))
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
    double factor = (pi.getLambda() + pj.getLambda());
    Vector2D temp = helper->spikyGrad(pi.getTmpPos() - pj.getTmpPos());
	Vector2D result =  temp * factor;

	return result;
}

void MapController::collision(int pID)
{
    static double BEDDING = 0.5 * MapGridSize;
    static double REBOUND = -0.5;
    
    Person &pi = people[pID];

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

    if (pi.getTmpPos().getX() > m_iWidth - BEDDING)
    {
        pi.setTmpPos(Vector2D(m_iWidth  - BEDDING, pi.getTmpPos().getY()));
        if (pi.getVelocity().getX() > 0.0)
        {
            pi.setVelocity(Vector2D(pi.getVelocity().getX() * REBOUND, pi.getVelocity().getY()));
        }
    }

    if (pi.getTmpPos().getY() > m_iHeight  - BEDDING)
    {
        pi.setTmpPos(Vector2D(pi.getTmpPos().getX(), m_iHeight - BEDDING));
        if (pi.getVelocity().getY() > 0.0)
        {
            pi.setVelocity(Vector2D(pi.getVelocity().getX(), pi.getVelocity().getY() * REBOUND));
        }
    }
}

bool MapController::isInMap(int x, int y)
{
    if (x < 0 || x >= m_iWidth) return false;
    if (y < 0 || y >= m_iHeight) return false;
    return true;
}
