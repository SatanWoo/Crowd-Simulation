#include "MapController.h"
#include "Terrain.h"
#include "TerrainFactory.h"
#include <GLUT/GLUT.h>
#include <math.h>
#include <deque>

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
    
    //flow = new Vector2D*[width];
	terrain = new Terrain *[width];
	for (int i = 0; i < width; i++)
    {
		terrain[i] = new Terrain [height];
        //flow[i] = new Vector2D[height];
	}
    
    for (int i = 0; i < width; i++)
    {
        for (int j = 0; j < height; j++)
        {
            terrain[i][j].setMap(this);
            
            if (rand() % 10 < 1)
            {
                terrain[i][j].init(INT_MAX);
            }
        }
    }
    
    destinationPoint.setX(rand() % m_iWidth * MapGridSize);
    destinationPoint.setY(rand() % m_iHeight * MapGridSize);

	helper = new MathHelper(1.5);
}

MapController::~MapController()
{
	for (int i = 0; i < m_iWidth; i++)
	{
		delete [] terrain[i];
        //delete [] flow[i];
        
		terrain[i] = NULL;
        //flow[i] = NULL;
	}

	delete [] terrain;
	terrain = NULL;
    
//    delete [] flow;
//    flow = NULL;

	delete [] people;
	people = NULL;

	delete helper;
	helper = NULL;
}

//void MapController::buildFlowField()
//{
//    for (int i = 0; i < m_iWidth; i++)
//    {
//        for (int j = 0; j < m_iHeight; j++)
//        {
//            if (terrain[i][j].getDistance() == INT_MAX) continue;
//            
//            Vector2D pos = Vector2D(i, j);
//            vector<Vector2D> neighbours = eightAdjacentNeighbours(pos);
//            
//            bool isMinFound = false;
//            Vector2D min;
//            int minDist = 0;
//            for (int k = 0; k < neighbours.size(); k++) {
//                int nx = neighbours[k].getX();
//                int ny = neighbours[k].getY();
//                
//                int dist = terrain[nx][ny].getDistance() - terrain[i][j].getDistance();
//                if (dist < minDist)
//                {
//                    isMinFound = true;
//                    minDist = dist;
//                    min = Vector2D(nx, ny);
//                }
//            }
//            
//            if (isMinFound)
//            {
//                flow[i][j] = (min - pos).normalize();
//            }
//        }
//    }
//}
//
//Vector2D MapController::steeringFromFlowFleid(int pID, Vector2D des)
//{
//    Person &pi = people[pID];
//    
//    Vector2D floor = pi.getPos().floor();
//    
//    int fx = floor.getX();
//    int fy = floor.getY();
//    
//    Vector2D f00 = flow[fx][fy];
//    Vector2D f01 = flow[fx][fy + 1];
//    Vector2D f10 = flow[fx + 1][fy];
//    Vector2D f11 = flow[fx + 1][fy + 1];
//    
//    double xWeight = pi.getPos().getX() - floor.getX();
//    Vector2D top = f00 * (1 - xWeight) + f10 *xWeight;
//    Vector2D bottom = f01 * (1 - xWeight) + f11 * xWeight;
//    
//    double yWeight = pi.getPos().getY() - floor.getY();
//    Vector2D direction = top * (1 - yWeight) + (bottom * yWeight).normalize();
//    
//    Vector2D desiredVelocity = direction * pi.getMaxSpeed();
//    
//    Vector2D velocityChange = desiredVelocity - pi.getVelocity();
//    return velocityChange * (pi.getMaxForce() / pi.getMaxSpeed());
//}

//void MapController::buildDijkstra()
//{
//    int desX = destinationPoint.getX() / MapGridSize;
//    int desY = destinationPoint.getY() / MapGridSize;
//    
//    deque<Vector2D> bfs;
//    bfs.push_back(Vector2D(desX, desY));
//    terrain[desX][desY].setDistance(0);
//    
//    while (!bfs.empty())
//    {
//        Vector2D head = bfs.front();
//        vector<Vector2D> neighbours = fourAdjacentNeighbours(head);
//        
//        int hx = head.getX();
//        int hy = head.getY();
//        
//        for (int i = 0; i < neighbours.size(); i++)
//        {
//            int nx = neighbours[i].getX();
//            int ny = neighbours[i].getY();
//            
//            if (terrain[nx][ny].getDistance() == -1)
//            {
//                terrain[nx][ny].setDistance(terrain[hx][hy].getDistance() + 1);
//            }
//        }
//        
//        bfs.pop_front();
//    }
//}
//
//vector<Vector2D> MapController::fourAdjacentNeighbours(const Vector2D &vec)
//{
//    
//    vector<Vector2D> result;
//    
//    static int dir[4][2] = {{-1, 0}, {1, 0}, {0, -1}, {0, 1}};
//    
//    int x = vec.getX();
//    int y = vec.getY();
//    for (int i = 0; i < 4; i++) {
//        int dx = x + dir[i][0];
//        int dy = y + dir[i][1];
//        
//        if (isInMap(dx, dy)) result.push_back(Vector2D(dx, dy));
//    }
//    
//    return result;
//}
//
//vector<Vector2D> MapController::eightAdjacentNeighbours(const Vector2D &vec)
//{
//    
//    vector<Vector2D> result;
//    
//    static int dir[8][2] = {{-1, 0}, {1, 0}, {0, -1}, {0, 1}, {-1, -1}, {-1, 1}, {1, -1}, {1, 1}};
//    
//    int x = vec.getX();
//    int y = vec.getY();
//    for (int i = 0; i < 8; i++) {
//        int dx = x + dir[i][0];
//        int dy = y + dir[i][1];
//        
//        if (isInMap(dx, dy)) result.push_back(Vector2D(dx, dy));
//    }
//    
//    return result;
//}
//


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
    glBegin(GL_QUADS);
    glColor3f(1.0f, 0.0f, 0.0f);
    for (int i = 0; i < m_iWidth; i++) {
        for (int j = 0; j < m_iHeight; j++) {
           
            double xPos = i * MapGridSize;
            double yPos = j * MapGridSize;
            
            if (terrain[i][j].obstacleCoefficient() == INT_MAX)
            {
                glVertex2d(xPos, yPos);
                glVertex2d(xPos + MapGridSize, yPos);
                glVertex2d(xPos + MapGridSize, yPos + MapGridSize);
                glVertex2d(xPos, yPos + MapGridSize);
            }
        }
    }
    glEnd();
    
    glPointSize(5);
    glColor3f(1.0f, 1.0f, 1.0f);
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
