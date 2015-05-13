#include "MapController.h"
#include "Terrain.h"
#include "TerrainFactory.h"
#include "MathLib.h"
#include "B2Vec2DHelper.h"

#include <GLUT/GLUT.h>
#include <math.h>
#include <deque>
#include <queue>

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
    
    flow = initializeVecField();
    
    avgVelocityField = initializeVecField();
    
    densityField = initializeFloatField();
    potentialField = initializeFloatField();
    discomfortField = initializeFloatField();
    
	terrain = new Terrain *[width];
    visited = new bool*[width];
    costField = new FourGrid*[width];
    speedField = new FourGrid*[width];
	for (int i = 0; i < width; i++)
    {
		terrain[i] = new Terrain [height];
        visited[i] = new bool [height];
        costField[i] = new FourGrid[height];
        speedField[i] = new FourGrid[height];
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
    cout << "Des is " << destinationPoint.x << "::" << destinationPoint.y << endl;
    
//    des.push_back(b2Vec2(rand() % m_iWidth * MapGridSize, rand() % m_iHeight * MapGridSize));
//    des.push_back(b2Vec2(rand() % m_iWidth * MapGridSize, rand() % m_iHeight * MapGridSize));

	helper = new MathHelper(MapGridSize);
}

b2Vec2** MapController::initializeVecField()
{
    b2Vec2** field = new b2Vec2*[m_iWidth];
    for (int i = 0; i < m_iWidth; i++)
    {
        field[i] = new b2Vec2[m_iHeight];
    }
    
    return field;
}

float32** MapController::initializeFloatField()
{
    float32 **field = new float32*[m_iWidth];
    for (int i = 0; i < m_iWidth; i++) {
        field[i] = new float32[m_iHeight];
    }
    return field;
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

void MapController::deinitializeField(float32 **field)
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
    deinitializeField(discomfortField);
    deinitializeField(avgVelocityField);
    
	for (int i = 0; i < m_iWidth; i++)
	{
		delete [] terrain[i];
        delete [] visited[i];
        delete [] costField[i];
        delete [] speedField[i];
        
		terrain[i] = NULL;
        visited[i] = NULL;
        costField[i] = NULL;
        speedField[i] = NULL;
	}

	delete [] terrain;
	terrain = NULL;
    
    delete [] visited;
    visited = NULL;
    
    delete [] costField;
    costField = NULL;
    
    delete [] speedField;
    speedField = NULL;
    
	delete [] people;
	people = NULL;

	delete helper;
	helper = NULL;
}

//void MapController::buildDijkstra()
//{
//    int desX = destinationPoint.x / MapGridSize;
//    int desY = destinationPoint.y / MapGridSize;
//    
//    deque<b2Vec2> bfs;
//    bfs.push_back(b2Vec2(desX, desY));
//    terrain[desX][desY].setDistance(0);
//    
//    while (!bfs.empty())
//    {
//        b2Vec2 head = bfs.front();
//        vector<b2Vec2> neighbours = fourAdjacentNeighbours(head);
//        
//        int hx = head.x;
//        int hy = head.y;
//        
//        for (int i = 0; i < neighbours.size(); i++)
//        {
//            int nx = neighbours[i].x;
//            int ny = neighbours[i].y;
//            
//            if (terrain[nx][ny].getDistance() == -1)
//            {
//                terrain[nx][ny].setDistance(terrain[hx][hy].getDistance() + 1);
//                bfs.push_back(b2Vec2(nx, ny));
//            }
//        }
//        
//        bfs.pop_front();
//    }
//}

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

b2Vec2 MapController::steeringFromSeparation(int pID)
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

b2Vec2 MapController::steeringFromAlignment(int pID)
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

b2Vec2 MapController::steeringFromCohesion(int pID)
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

b2Vec2 MapController::steeringFromAvoidance(int pID)
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
    Person &pi = people[pID];
    b2Vec2 flowForce = pi.flowForce;
    //b2Vec2 seekForce = steeringFromSeek(pID, destinationPoint);
    b2Vec2 separationForce = steeringFromSeparation(pID);
    b2Vec2 alignForce = steeringFromAlignment(pID);
    b2Vec2 cohesionForce = steeringFromCohesion(pID);
    
    //Vector2D lowCostForce = steeringFromFlowFleid(pID, destinationPoint);
    b2Vec2 appliedForce = flowForce  + separationForce * 0.3 + alignForce * 0.03 + cohesionForce * 0.05;
    
    float32 l = appliedForce.Length();
    if (l > pi.getMaxForce())
    {
        appliedForce = appliedForce * (pi.getMaxForce() / l);
    }
    
    return appliedForce;
}

void MapController::update()
{
    updateContinuumCrowdData();
//    
    for (int i = 0; i < m_iCount; i++) {
        Person &pi = people[i];
        pi.flock(&MapController::flock);
    }
//
    for (int i = 0; i < m_iCount; i++) {
        Person &pi = people[i];
        pi.steer();
    }
    
//    world->Step(m_dTimeStep, 10, 10);
//    world->ClearForces();
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

// Continnum Crowd
void MapController::updateContinuumCrowdData()
{
    ccClearBuffers();
    ccCalculateDensityAndAverageSpeed();
    ccCalculateUnitCostField();
//
    ccClearPotentialField();
    ccPotentialFieldEikonalFill(destinationPoint);
    ccGenerateFlowField(); //TODO: This does not use the way of calculating described in the paper (I think)
//    
    for (int i = m_iCount - 1; i >= 0; i--) {
        Person &pi = people[i];
        pi.flowForce = steeringFromFlowFleid(i, destinationPoint);;
    }
}

void MapController::ccClearBuffers()
{
    for (int i = 0; i < m_iWidth; i++)
    {
        for (int j = 0; j < m_iHeight; j++)
        {
            densityField[i][j] = 0;
            discomfortField[i][j] = 0;
            avgVelocityField[i][j].SetZero();
        }
    }
}

void MapController::ccCalculateDensityAndAverageSpeed()
{
    float32 perAgentDensity = 1.0;
    
    for (int i = 0; i < m_iCount; i++)
    {
        Person &pi = people[i];
        
        b2Vec2 floor = B2Vec2DHelper::floorV(pi.getPosition());
        float32 xWeight = pi.getPosition().x - floor.x;
        float32 yWeight = pi.getPosition().y - floor.y;
        
        //top left
        if (isInMap(floor.x, floor.y)) {
            ccAddDensity(floor.x, floor.y, pi.getVelocity(), perAgentDensity * (1 - xWeight) * (1 - yWeight));
        }
        //top right
        if (isInMap(floor.x + 1, floor.y)) {
            ccAddDensity(floor.x + 1, floor.y, pi.getVelocity(), perAgentDensity * (xWeight) * (1 - yWeight));
        }
        //bottom left
        if (isInMap(floor.x, floor.y + 1)) {
            ccAddDensity(floor.x, floor.y + 1, pi.getVelocity(), perAgentDensity * (1 - xWeight) * (yWeight));
        }
        //bottom right
        if (isInMap(floor.x + 1, floor.y + 1)) {
            ccAddDensity(floor.x + 1, floor.y + 1, pi.getVelocity(), perAgentDensity * (xWeight) * (yWeight));
        }
    }
    
    for (int i = 0; i < m_iWidth; i++)
    {
        for (int j = 0; j < m_iHeight; j++)
        {
            b2Vec2& velocity = avgVelocityField[i][j];
            float32 density = densityField[i][j];
            if (density > 0) {
                velocity *= (1/density);
            }
        }
    }
}

void MapController::ccAddDensity(int x, int y, const b2Vec2& vec, float32 weight)
{
    densityField[x][y] += weight;
    
    b2Vec2& v = avgVelocityField[x][y];
    v.x += vec.x * weight;
    v.y += vec.y * weight;
}

void MapController::ccCalculateUnitCostField()
{
    float32 densityMin = 0.5;
    float32 densityMax = 0.8;
    
    //Weights for formula (4) on page 4
    float32 lengthWeight = 1;
    float32 timeWeight = 1;
    float32 discomfortWeight = 1;
    
    for (int i = 0; i < m_iWidth; i++) {
        for (int j = 0; j < m_iHeight; j++) {
            
            //foreach direction we can leave that cell
            for (int dir = 0; dir < 4; dir++) {
                int targetX = i + fourDir[dir][0];
                int targetY = j + fourDir[dir][1];
                
                if (!isInMap(targetX, targetY)) {
                    speedField[i][j].value[dir] = INT_MAX;
                    continue;
                }
                
                float32 veloX = fourDir[dir][0] * avgVelocityField[targetX][targetY].x;
                float32 veloY = fourDir[dir][1] * avgVelocityField[targetX][targetY].y;
                
                //Get the only speed value as one will be zero
                // this is like speedVecX != 0 ? : speedVecX : speedVecY
                float32 flowSpeed = fourDir[dir][0] != 0 ? veloX : veloY;
                
                float32 density = densityField[targetX][targetY];
                float32 discomfort = discomfortField[targetX][targetY];
                
                if (density >= densityMax) {
                    speedField[i][j].value[dir] = flowSpeed;
                } else if (density <= densityMin) {
                    speedField[i][j].value[dir] = 4;
                } else {
                    //medium speed
                    speedField[i][j].value[dir] = 4 - (density - densityMin) / (densityMax - densityMin) * (4 - flowSpeed);
                }
                
                //we're going to divide by speed later, so make sure it's not zero
                float32 speed = speedField[i][j].value[dir];
                float32 threshold = 0.001;
                speedField[i][j].value[dir] = min(threshold, speed);
                
                //Work out the cost to move in to the destination cell
                costField[i][j].value[dir] = (speedField[i][j].value[dir] * lengthWeight + timeWeight + discomfortWeight * discomfort) / speedField[i][j].value[dir];
            }
        }
    }
}

void MapController::ccClearPotentialField()
{
    for (int i = 0; i < m_iWidth; i++) {
        for (int j = 0; j < m_iHeight; j++) {
            potentialField[i][j] = INT_MAX;
        }
    }
}

void MapController::ccGenerateFlowField()
{
    for (int i = 0; i < m_iWidth; i++) {
        for (int j = 0; j < m_iHeight; j++) {
            flow[i][j].SetZero();
        }
    }
    
    for (int i = 0; i < m_iWidth; i++)
    {
        for (int j = 0; j < m_iHeight; j++)
        {
            if (potentialField[i][j] == INT_MAX) continue;
            
            bool isMinFound = false;
            b2Vec2 min;
            int minDist = INT_MAX;
            
            for (int d = 0; d < 8; d++) {
                if (isInMap(i + eightDir[d][0], j + eightDir[d][1])) {
                    float32 dist = potentialField[i + eightDir[d][0]][j + eightDir[d][1]];
                    
                    if (dist < minDist) {
                        min.x = eightDir[d][0];
                        min.y = eightDir[d][1];
                        minDist = dist;
                        
                        isMinFound = true;
                    }
                }
            }
            
            if (isMinFound) {
                flow[i][j].Set(min.x, min.y);
                flow[i][j].Normalize();
            }
        }
    }
}

void MapController::ccPotentialFieldEikonalFill(b2Vec2 des)
{
    for (int i = 0; i < m_iWidth; i++) {
        for (int j = 0; j < m_iHeight; j++) {
            visited[i][j] = false;
        }
    }
    
    int candidatesCount = 0;
    
    CostNode desNode;
    desNode.cost = 0;
    desNode.point.x = des.x / MapGridSize;
    desNode.point.y = des.y / MapGridSize;
    
    priority_queue<CostNode> pQueue;
    pQueue.push(desNode);
    
    cout << "Width" << m_iWidth << "Height" << m_iHeight << endl;
    
    while (!pQueue.empty()) {
        candidatesCount++;
        CostNode at = pQueue.top();
        pQueue.pop();
        
        int x = at.point.x;
        int y = at.point.y;
        
        cout << "Point is " << x << "::" << y << endl;
        
        if (potentialField[x][y] >= at.cost && !visited[x][y]) {
            
            potentialField[x][y] = at.cost;
            visited[x][y] = true;
            
            for (int i = 0; i < 4; i++) {
                int toX = x + fourDir[i][0];
                int toY = y + fourDir[i][1];
                if (isInMap(toX, toY)) {
                    //Cost to go from our target cell to the start
                    //Our cost + cost of moving from the target to us
                    float32 toCost = at.cost + costField[toX][toY].value[(i + 2) % 4];
                    
                    //If we present a better path, overwrite the cost and queue up going to that cell
                    if (toCost < potentialField[toX][toY]) {
                        //console.log('Queueing ' + toX + ', ' + toY + ' @ ' + toCost);
                        potentialField[toX][toY] = toCost;
                        visited[x][y] = false;
                        
                        CostNode toP ;
                        toP.point.x = toX;
                        toP.point.y = toY;
                        toP.cost = toCost;
                        pQueue.push(toP);
                    }
                }
            }
        }
    }
}