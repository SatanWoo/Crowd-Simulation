#include "MapController.h"
#include "Terrain.h"
#include "TerrainFactory.h"
#include "MathLib.h"
#include "B2Vec2DHelper.h"

#include <GLUT/GLUT.h>
#include <math.h>
#include <deque>

const double MapController::restDensity = 1.0;
const double MapController::MapGridSize = 1.5;

static int fourDir[4][2] = {{-1, 0}, {1, 0}, {0, -1}, {0, 1}};
static int eightDir[8][2] = {{-1, 0}, {1, 0}, {0, -1}, {0, 1}, {-1, -1}, {-1, 1}, {1, -1}, {1, 1}};

MapController::MapController(int width, int height, int count, double timeStep)
{
	m_iWidth = width;
	m_iHeight = height;
	m_iCount = count;
	m_dTimeStep = timeStep;
    
    world = new b2World(b2Vec2_zero, true);

	people = new Person[count];
    srand( (unsigned)time(NULL));
    
	for (int i = 0; i < count; i++)
	{
        b2Vec2 pos = b2Vec2(MapGridSize, rand()% m_iHeight * MapGridSize);
        people[i].setMap(this);
        people[i].init(i, 0, pos);
        people[i].initBodyDef();
	}
    
    for (int i = count / 2; i < count; i++)
    {
        b2Vec2 pos = b2Vec2((m_iWidth - 1) * MapGridSize, rand()% m_iHeight * MapGridSize);
        people[i].setMap(this);
        people[i].init(i, 1, pos);
        people[i].initBodyDef();
    }
    
    initializeField(flow);
    initializeField(potentialField);
    initializeField(costField);
    initializeField(discomfortField);
    initializeField(speedField);
    initializeField(avgVelocityField);
    
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
//            if (rand() % 10 < 1)
//            {
//                terrain[i][j].init(INT_MAX);
//                terrain[i][j].setDistance(INT_MAX);
//            }
        }
    }
    
    destinationPoint = b2Vec2(rand() % m_iWidth * MapGridSize, rand() % m_iHeight * MapGridSize);
    
//    des.push_back(b2Vec2(rand() % m_iWidth * MapGridSize, rand() % m_iHeight * MapGridSize));
//    des.push_back(b2Vec2(rand() % m_iWidth * MapGridSize, rand() % m_iHeight * MapGridSize));

	helper = new MathHelper(MapGridSize);
    
    buildDijkstra();
    buildFlowField();
}

void MapController::initializeField(b2Vec2 **field)
{
    field = new b2Vec2*[m_iWidth];
    for (int i = 0; i < m_iWidth; i++)
    {
        field[i] = new b2Vec2[m_iHeight];
    }
}

void MapController::deinitializeField(b2Vec2 **field)
{
    for (int i = 0; i < m_iWidth; i++) {
        delete [] field[i];
        field[i] = NULL;
    }
    
    delete [] field;
    field = NULL;
}

MapController::~MapController()
{
    deinitializeField(flow);
    deinitializeField(potentialField);
    deinitializeField(costField);
    deinitializeField(discomfortField);
    deinitializeField(speedField);
    deinitializeField(avgVelocityField);
    
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

void MapController::buildDijkstra()
{
    int desX = destinationPoint.x / MapGridSize;
    int desY = destinationPoint.y / MapGridSize;
    
    deque<b2Vec2> bfs;
    bfs.push_back(b2Vec2(desX, desY));
    terrain[desX][desY].setDistance(0);
    
    while (!bfs.empty())
    {
        b2Vec2 head = bfs.front();
        vector<b2Vec2> neighbours = fourAdjacentNeighbours(head);
        
        int hx = head.x;
        int hy = head.y;
        
        for (int i = 0; i < neighbours.size(); i++)
        {
            int nx = neighbours[i].x;
            int ny = neighbours[i].y;
            
            if (terrain[nx][ny].getDistance() == -1)
            {
                terrain[nx][ny].setDistance(terrain[hx][hy].getDistance() + 1);
                bfs.push_back(b2Vec2(nx, ny));
            }
        }
        
        bfs.pop_front();
    }
}

void MapController::buildFlowField()
{
    for (int i = 0; i < m_iWidth; i++)
    {
        for (int j = 0; j < m_iHeight; j++)
        {
            if (terrain[i][j].getDistance() == INT_MAX) continue;
            
            b2Vec2 pos = b2Vec2(i, j);
            vector<b2Vec2> neighbours = eightAdjacentNeighbours(pos);
            
            bool isMinFound = false;
            b2Vec2 min;
            int minDist = 0;
            for (int k = 0; k < neighbours.size(); k++) {
                int nx = neighbours[k].x;
                int ny = neighbours[k].y;
                
                int dist = terrain[nx][ny].getDistance() - terrain[i][j].getDistance();
                if (dist < minDist)
                {
                    isMinFound = true;
                    minDist = dist;
                    min = b2Vec2(nx, ny);
                }
            }
            
            if (isMinFound)
            {
                flow[i][j] = min - pos;
                flow[i][j].Normalize();
            }
        }
    }
}

b2Vec2 MapController::steeringFromFlowFleid(int pID, b2Vec2 des)
{
    Person &pi = people[pID];
    
    b2Vec2 floor = B2Vec2DHelper::floorV(pi.getPosition());
    
    int fx = floor.x / MapGridSize;
    int fy = floor.y / MapGridSize;
    
    b2Vec2 f00 = isAccessible(fx, fy) ? flow[fx][fy] : b2Vec2_zero;
    b2Vec2 f01 = isAccessible(fx, fy + 1) ? flow[fx][fy + 1] : b2Vec2_zero;
    b2Vec2 f10 = isAccessible(fx + 1, fy) ? flow[fx + 1][fy] : b2Vec2_zero;
    b2Vec2 f11 = isAccessible(fx + 1, fy + 1) ? flow[fx + 1][fy + 1] : b2Vec2_zero;
    
    double xWeight = pi.getPosition().x - floor.x;
    b2Vec2 top = f00 * (1 - xWeight) + f10 * xWeight;
    b2Vec2 bottom = f01 * (1 - xWeight) + f11 * xWeight;
    
    double yWeight = pi.getPosition().y - floor.y;
    b2Vec2 direction = top * (1 - yWeight) + (bottom * yWeight);
    direction.Normalize();
    
    if (isnan(direction.LengthSquared())) {
        direction.SetZero();
        return direction;
    }
    
    return steeringTowards(pID, direction);
}

b2Vec2 MapController::steeringFromSeek(int pID, b2Vec2 des)
{
    Person &pi = people[pID];
    if (des.x == pi.getPosition().x && des.y == pi.getPosition().y) return b2Vec2_zero;
    
    b2Vec2 distance = des - pi.getPosition();
    b2Vec2 desiredSpeed = distance * (pi.getMaxSpeed() / distance.Length());
    b2Vec2 velocityChange = desiredSpeed - pi.getLinearVelocity();
    
    return velocityChange * (pi.getMaxForce() / pi.getMaxSpeed());
}

b2Vec2 MapController::steeringFromSeparation(int pID, b2Vec2 des)
{
    Person &pi = people[pID];
    
    int neighCount = 0;
    b2Vec2 totalForce = b2Vec2_zero;
    
    for (int i = 0; i < m_iCount; i++) {
        if (i == pID) continue;
        
        Person &pn = people[i];
        double distance = B2Vec2DHelper::distanceTo(pi.getPosition(), pn.getPosition());
        if (distance > 0 && distance < pi.getMinSeparation())
        {
            neighCount += 1;
            b2Vec2 pushForce = pi.getPosition() - pn.getPosition();
            float32 length = pushForce.Normalize();
            float32 r = pi.getRadius() + pn.getRadius();
            totalForce += pushForce * (1 -  ((length - r) / (pi.getMinSeparation() - r)));
        }
    }
    
    if (neighCount == 0) return totalForce;
    
    return totalForce * (pi.getMaxForce() / neighCount);
}

b2Vec2 MapController::steeringFromAlignment(int pID, b2Vec2 des)
{
    Person &pi = people[pID];
    int neighCount = 0;
    b2Vec2 avergaeHeading = b2Vec2_zero;
    
    for (int i = 0; i < m_iCount; i++) {
        Person &pn = people[i];
        double distance = B2Vec2DHelper::distanceTo(pi.getPosition(), pn.getPosition());
        if (distance < pi.getMaxCohesion() && pn.getLinearVelocity().Length() > 0 && pi.getGroupID() == pn.getGroupID())
        {
            neighCount += 1;
            b2Vec2 head(pn.getLinearVelocity());
            head.Normalize();
            
            avergaeHeading += head;
        }
    }
    
    if (neighCount == 0) return avergaeHeading;

    return steeringTowards(pID, avergaeHeading);
}

b2Vec2 MapController::steeringFromCohesion(int pID, b2Vec2 des)
{
    Person &pi = people[pID];
    b2Vec2 centerOfMass = b2Vec2_zero;
    int neighCount = 0;
    
    for (int i = 0; i < m_iCount; i++) {
        if (i == pID) continue;
        
        Person &pn = people[i];
        if (pn.getGroupID() != pi.getGroupID()) continue;
        
        double distance = B2Vec2DHelper::distanceTo(pi.getPosition(), pn.getPosition());
        if (distance < pi.getMaxCohesion())
        {
            centerOfMass += pn.getPosition();
            neighCount++;
        }
    }
    
    if (neighCount == 0) return b2Vec2_zero;
    centerOfMass *=  1 / neighCount;
    return steeringFromSeek(pID, centerOfMass);
}

b2Vec2 MapController::steeringFromAvoidance(int pID, b2Vec2 des)
{
    return b2Vec2_zero;
}

b2Vec2 MapController::steeringFromLowestCost(int pID, b2Vec2 des)
{
    Person &pi = people[pID];
    
    if (isnan(pi.getPosition().x) || isnan(pi.getPosition().y)) return b2Vec2_zero;
    if (pi.getLinearVelocity().Length() == 0) return b2Vec2_zero;
    
    b2Vec2 floor = pi.getPosition();
    int fx = floor.x / MapGridSize;
    int fy = floor.y / MapGridSize;
    
    float f00 = isAccessible(fx, fy) ? terrain[fx][fy].getDistance() : INT_MAX;
    float f01 = isAccessible(fx, fy + 1) ? terrain[fx][fy + 1].getDistance() : INT_MAX;
    float f10 = isAccessible(fx + 1, fy) ? terrain[fx + 1][fy].getDistance() : INT_MAX;
    float f11 = isAccessible(fx + 1, fy + 1) ? terrain[fx + 1][fy + 1].getDistance() : INT_MAX;
    
    float minVal = MathLib::min4(f00, f01, f10, f11);
    vector<Vector2D> minCoords;
    
    if (minVal == f00)
    {
        minCoords.push_back(Vector2D(fx, fy));
    }
    if (minVal == f01)
    {
        minCoords.push_back(Vector2D(fx, fy+1));
    }
    if (minVal == f10)
    {
        minCoords.push_back(Vector2D(fx + 1, fy));
    }
    if (minVal == f11)
    {
        minCoords.push_back(Vector2D(fx + 1, fy + 1));
    }
    
    b2Vec2 currentDirection = pi.getLinearVelocity();
    b2Vec2  desireDirection;
    minVal = INT_MAX;
    
//    for (int i = 0; i < minCoords.size(); i++)
//    {
//        b2Vec2 directionTo = (minCoords[i] - pi.getPos()).normalize();
//        float length = (directionTo - currentDirection).squaredLength();
//        if (length < minVal)
//        {
//            minVal = length;
//            desireDirection = directionTo;
//        }
//    }
    
    return steeringTowards(pID, desireDirection);
}

b2Vec2 MapController::steeringTowards(int pID, b2Vec2 desiredDirection)
{
    Person &pi = people[pID];
    b2Vec2 desiredVelocity = desiredDirection * pi.getMaxSpeed();
    b2Vec2 velocityChange = desiredVelocity - pi.getLinearVelocity();
    
    return velocityChange * (pi.getMaxForce() / pi.getMaxSpeed());
}

vector<b2Vec2> MapController::fourAdjacentNeighbours(const b2Vec2 &vec)
{
    vector<b2Vec2> result;
    
    
    int x = vec.x;
    int y = vec.y;
    for (int i = 0; i < 4; i++) {
        int dx = x + fourDir[i][0];
        int dy = y + fourDir[i][1];
        
        if (isInMap(dx, dy)) result.push_back(b2Vec2(dx, dy));
    }
    
    return result;
}

vector<b2Vec2> MapController::eightAdjacentNeighbours(const b2Vec2 &vec)
{
    // 参数 Vec 已经是基于索引得了
    vector<b2Vec2> result;
    
    int x = vec.x;
    int y = vec.y;
    
    bool up = isAccessible(x, y - 1);
    bool down = isAccessible(x, y + 1);
    bool left = isAccessible(x - 1, y);
    bool right = isAccessible(x + 1, y);
    
    if (left)
    {
        result.push_back(b2Vec2(x - 1, y));
        if (up && isAccessible(x - 1, y - 1))
        {
            result.push_back(b2Vec2(x - 1, y - 1));
        }
    }
    
    if (right)
    {
        result.push_back(b2Vec2(x + 1, y));
        if (down && isAccessible(x + 1, y + 1))
        {
            result.push_back(b2Vec2(x + 1, y + 1));
        }
    }
    
    if (up)
    {
        result.push_back(b2Vec2(x, y - 1));
        if (right && isAccessible(x + 1, y - 1))
        {
            result.push_back(b2Vec2(x + 1, y - 1));
        }
    }
    
    if (down)
    {
        result.push_back(b2Vec2(x, y + 1));
        if (left && isAccessible(x - 1, y + 1))
        {
            result.push_back(b2Vec2(x - 1, y + 1));
        }
    }
    
    return result;
}

b2Vec2 MapController::flock(int pID)
{
    //b2Vec2 flowForce = steeringFromFlowFleid(pID, destinationPoint);
    b2Vec2 flowForce = b2Vec2_zero;
    b2Vec2 seekForce = steeringFromSeek(pID, destinationPoint);
    b2Vec2 separationForce = steeringFromSeparation(pID, destinationPoint);
    b2Vec2 alignForce = steeringFromAlignment(pID, destinationPoint);
    b2Vec2 cohesionForce = steeringFromCohesion(pID, destinationPoint);
    
    //Vector2D lowCostForce = steeringFromFlowFleid(pID, destinationPoint);
    b2Vec2 appliedForce = flowForce + seekForce + separationForce * 0.3 + alignForce * 0.05 + cohesionForce * 0.05;
    
    float32 l = appliedForce.Length();
    if (l > people[pID].getMaxForce())
    {
        appliedForce = appliedForce * (people[pID].getMaxForce() / l);
    }
    
    return appliedForce;
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
    
    world->Step(m_dTimeStep, 10, 10);
    world->ClearForces();
}

///////////
//void MapController::movePerson(b2Vec2 old, b2Vec2 cur, int pID)
//{
//	int oldX = old.x / MapGridSize;
//	int oldY = old.y / MapGridSize;
//    if (isInMap(oldX, oldY))
//        terrain[oldX][oldY].removePerson(pID);
//
//	int curX = cur.x / MapGridSize;
//	int curY = cur.y / MapGridSize;
//    if (isInMap(curX, curY))
//        terrain[curX][curY].addPerson(pID);
//}
//
//std::vector<int> MapController::findNeighbours(int pID)
//{
//	static int direction[9][2] = 
//	{
//		{-1,1}, {-1, 0}, {-1, -1}, 
//		{0, 1}, {0, 0}, {0, -1}, 
//		{1, 1}, {1, 0}, {1, -1}
//	};
//    
//	Person &p = people[pID];
//	int cellX = p.getTmpPos().x / MapGridSize;
//	int cellY = p.getTmpPos().y / MapGridSize;
//
//	std::vector<int> result;
//	for (int i = 0; i < 9; i++)
//	{
//		int curX = cellX + direction[i][0];
//        int curY = cellY + direction[i][1];
//        
//        if (!isInMap(curX, curY)) continue;
//        
//		Terrain &curUnit = terrain[curX][curY];
//
//		std::vector<int> temp = curUnit.filterPeople(&MapController::filterNeightbours, pID);
//        result.insert(result.end(), temp.begin(), temp.end());
//	}
//
//	return result;
//}

//bool MapController::filterNeightbours(int neighborID, int pID)
//{
//	Person &n = people[neighborID];
//	Person &p = people[pID];
//	double distance = (p.getTmpPos() - n.getTmpPos()).Length();
//    
//	return distance < MapGridSize * MapGridSize;
//}
//
//double MapController::density(int neighbourID, int pID)
//{
//	Person &pj = people[neighbourID];
//	Person &pi = people[pID];
//
//	return pj.getMass() * helper->poly6(pi.getTmpPos() - pj.getTmpPos());
//}
//
//Vector2D MapController::lamda(int neighbourID, int pID)
//{
//	Person &pj = people[neighbourID];
//	Person &pi = people[pID];
//
//	return helper->spikyGrad(pi.getTmpPos() - pj.getTmpPos());
//}
//
//Vector2D MapController::deltaP(int neighbourID, int pID)
//{
//	Person &pj = people[neighbourID];
//	Person &pi = people[pID];
//    double factor = (pi.getLambda() + pj.getLambda());
//    Vector2D temp = helper->spikyGrad(pi.getTmpPos() - pj.getTmpPos());
//	Vector2D result =  temp * factor;
//
//	return result;
//}
//
//void MapController::collision(int pID)
//{
//    static double BEDDING = 0.5 * MapGridSize;
//    static double REBOUND = -0.5;
//    
//    Person &pi = people[pID];
//
//    if (pi.getTmpPos().getX() < 0.0 + BEDDING)
//    {
//        pi.setTmpPos(Vector2D(BEDDING, pi.getTmpPos().getY()));
//        if (pi.getVelocity().getX() < 0.0)
//        {
//            pi.setVelocity(Vector2D(pi.getVelocity().getX() * REBOUND, pi.getVelocity().getY()));
//        }
//    }
//
//    if (pi.getTmpPos().getY() < 0.0 + BEDDING)
//    {
//        pi.setTmpPos(Vector2D(pi.getTmpPos().getX(), BEDDING));
//        if (pi.getVelocity().getY() < 0.0)
//        {
//            pi.setVelocity(Vector2D(pi.getVelocity().getX(), pi.getVelocity().getY() * REBOUND));
//        }
//    }
//
//    if (pi.getTmpPos().getX() > m_iWidth - BEDDING)
//    {
//        pi.setTmpPos(Vector2D(m_iWidth  - BEDDING, pi.getTmpPos().getY()));
//        if (pi.getVelocity().getX() > 0.0)
//        {
//            pi.setVelocity(Vector2D(pi.getVelocity().getX() * REBOUND, pi.getVelocity().getY()));
//        }
//    }
//
//    if (pi.getTmpPos().getY() > m_iHeight  - BEDDING)
//    {
//        pi.setTmpPos(Vector2D(pi.getTmpPos().getX(), m_iHeight - BEDDING));
//        if (pi.getVelocity().getY() > 0.0)
//        {
//            pi.setVelocity(Vector2D(pi.getVelocity().getX(), pi.getVelocity().getY() * REBOUND));
//        }
//    }
//}

bool MapController::isInMap(int x, int y)
{
    if (x < 0 || x >= m_iWidth) return false;
    if (y < 0 || y >= m_iHeight) return false;
    return true;
}

bool MapController::isAccessible(int x, int y)
{
    if (!isInMap(x, y)) return false;
    if (terrain[x][y].getDistance() == INT_MAX) return false;
    
    return true;
}

void MapController::render()
{
    glPointSize(10);
    glBegin(GL_POINTS);
    glColor4f(0.0, 1.0, 0.0, 1.0);
    glVertex2d(destinationPoint.x, destinationPoint.y);
    glEnd();
    
    glLineWidth(1);
    glBegin(GL_LINES);
    
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
    glBegin(GL_POINTS);
    for (int i = 0; i < m_iCount; i++) {
        
        Person &p = people[i];
        if (p.getGroupID() == 0) {
            glColor3f(1.0f, 0.0f, 1.0f);
        } else {
            glColor3f(1.0f, 1.0f, 1.0f);
        }
        p.render();
    }
    glEnd();
}
