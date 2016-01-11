#include "MapController.h"
#include "MathLib.h"
#include "Agent.h"
#include "B2Vec2DHelper.h"

#include <GLUT/GLUT.h>
#include <math.h>
#include <deque>
#include <queue>

using namespace std;

const double MapController::restDensity = 1.0;
const double MapController::MapGridSize = 32;

MapController::MapController(int width, int height, int count, double timeStep)
{
	m_iWidth = width;
	m_iHeight = height;
	m_dTimeStep = timeStep;
    
    world = new b2World(b2Vec2_zero, true);

    srand((unsigned)time(NULL));
    
    b2Vec2 v1(m_iWidth - 2, m_iHeight / 2);
    destinationPoints.push_back(v1);

    for (int yPos = 1; yPos < m_iHeight - 1; yPos++) {
        for (int i =0 ; i < 3; i++) {
            Agent p1(b2Vec2(i % 3, yPos), 0);
            p1.initBodyDef(world);
            agents.push_back(p1);
        }
    }
    
    double k = 100;
    int K = k;

    for (int i = 0; i < 30; ++i) {
        double xRand = double(rand() % K) / k;
        double yRand = double(rand() % K) / k;
        
        int x = 1 + floor(xRand * (m_iWidth - 3));
        int y = floor(yRand * (m_iHeight - 2));
        
        obstacles.push_back(b2Vec2(x, y));
    }
    
    for (int i = 0; i < obstacles.size(); i++) {
        b2Vec2 pos = obstacles[i];
        
        //Create a physics body for the agent
        b2FixtureDef *fixDef = new b2FixtureDef();
        b2BodyDef *bodyDef = new b2BodyDef();
        
        fixDef->density = 1.0;
        fixDef->friction = 0.5;
        fixDef->restitution = 0.2;
        fixDef->shape = new b2PolygonShape();
        
        b2PolygonShape *shape = (b2PolygonShape *)fixDef->shape;
        shape->SetAsBox(0.5, 0.5);
        
        bodyDef->type = b2_staticBody;
        bodyDef->position.Set(pos.x, pos.y);
        
        world->CreateBody(bodyDef)->CreateFixture(fixDef);
    }
    
    flow = initializeVecField();
    dijkstra = initializeFloatField();
    lost = new bool*[width];
	for (int i = 0; i < width; i++)
    {
        lost[i] = new bool [height];
	}
    
    ccGenerateDijkstraField();
    ccGenerateFlowField();
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

void MapController::deinitializeField(bool **field)
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
    deinitializeField(dijkstra);
    deinitializeField(lost);
}

#pragma mark -
void MapController::render()
{
//    for (int i = 0; i < agents.size(); i++)
//    {
//        Agent &agent = agents[i];
//        
//        glPushMatrix();
//        glTranslatef(agent.getPosition().x, 0.0f, agent.getPosition().y);
//        glutSolidSphere(0.5, 5, 5);
//        glColor4f(1, .5, .5, 1); // pink head
//        glTranslatef(0.0f, 0.5f, 0.0f);
//        glutSolidSphere(.25, 5, 5);
//        glPopMatrix();
//    }
    
    int xOffset = 0;
    int yOffset = 0;
    
    
    
    glPointSize(5);
    glBegin(GL_POINTS);
    for (int i = 0; i < agents.size(); i++) {
        
        Agent &agent = agents[i];
        if (agent.group == 0) {
            glColor3f(1.0f, 0.0f, 1.0f);
        } else {
            glColor3f(1.0f, 1.0f, 1.0f);
        }
        glVertex3f(agent.getPosition().x * MapGridSize + 0.5 * MapGridSize - yOffset,  0, agent.getPosition().y * MapGridSize + 0.5 * MapGridSize - yOffset);
    }
    glEnd();
    
    glBegin(GL_QUADS);
    glColor3f(1.0f, 0.0f, 0.0f);
    
    int height = 150;
    
    for (int i = 0 ; i < obstacles.size(); i++) {
        b2Vec2 o = obstacles[i];
        
        int x = o.x;
        int y = o.y;
        
        glVertex3f(x * MapGridSize - xOffset, 0, y * MapGridSize - yOffset);
        glVertex3f((x + 1) * MapGridSize - xOffset, 0, y * MapGridSize - yOffset);
        glVertex3f((x + 1) * MapGridSize - xOffset, 0, (y + 1) * MapGridSize - yOffset);
        glVertex3f(x * MapGridSize - xOffset, 0, (y + 1) * MapGridSize - yOffset);
        
        glVertex3f(x * MapGridSize - xOffset, height, y * MapGridSize - yOffset);
        glVertex3f((x + 1) * MapGridSize - xOffset, height, y * MapGridSize - yOffset);
        glVertex3f((x + 1) * MapGridSize - xOffset, height, (y + 1) * MapGridSize - yOffset);
        glVertex3f(x * MapGridSize - xOffset, height, (y + 1) * MapGridSize - yOffset);
        
        glVertex3f(x * MapGridSize - xOffset, 0, y * MapGridSize - yOffset);
        glVertex3f((x + 1) * MapGridSize - xOffset, 0, y * MapGridSize - yOffset);
        glVertex3f((x + 1) * MapGridSize - xOffset, height, y * MapGridSize - yOffset);
        glVertex3f(x * MapGridSize - xOffset, height, y * MapGridSize - yOffset);
        
        glVertex3f(x * MapGridSize - xOffset, 0, y * MapGridSize - yOffset);
        glVertex3f(x * MapGridSize - xOffset, 0, (y + 1) * MapGridSize - yOffset);
        glVertex3f(x * MapGridSize - xOffset, height, (y + 1) * MapGridSize - yOffset);
        glVertex3f(x * MapGridSize - xOffset, height, y * MapGridSize - yOffset);
        
        glVertex3f((x + 1) * MapGridSize - xOffset, 0, (y + 1) * MapGridSize - yOffset);
        glVertex3f((x + 1) * MapGridSize - xOffset, 0, y * MapGridSize - yOffset);
        glVertex3f((x + 1) * MapGridSize - xOffset, height, y * MapGridSize - yOffset);
        glVertex3f((x + 1) * MapGridSize - xOffset, height, (y + 1) * MapGridSize - yOffset);
    }
    glEnd();
    
    
}

void MapController::update()
{
    for (int i = agents.size() - 1; i >= 0; i--) {
        Agent &agent = agents[i];
        
        b2Vec2 ff  = steeringBehaviourFlowField(agent);
        b2Vec2 sep = steeringBehaviourSeparation(agent);
        b2Vec2 alg = steeringBehaviourAlignment(agent);
        b2Vec2 coh = steeringBehaviourCohesion(agent);
        
        agent.force = ff + sep * 2 + alg * 0.5 + coh * 0.2;
        
        float32 lengthSquared =  agent.force.LengthSquared();
        if (lengthSquared > agent.maxForceSquared) {
            agent.force *= (agent.maxForce / sqrt(lengthSquared));
        }
    }
    
    //Move agents based on forces being applied (aka physics)
    for (int i = agents.size() - 1; i >= 0; i--) {
        Agent &agent = agents[i];
        agent.body->ApplyLinearImpulse(agent.force * m_dTimeStep, agent.getPosition());
    }
    
    world->Step(m_dTimeStep, 10, 10);
    world->ClearForces();
}

b2Vec2 MapController::steeringBehaviourFlowField(Agent &agent)
{
    b2Vec2 floor = B2Vec2DHelper::floorV(agent.getPosition());
    
    int x = floor.x;
    int y = floor.y;
    
    b2Vec2 f00 = isValid(x, y) ? flow[x][y] : b2Vec2_zero;
    b2Vec2 f01 = isValid(x, y + 1) ? flow[x][y + 1] : b2Vec2_zero;
    b2Vec2 f10 = isValid(x + 1, y) ? flow[x + 1][y] : b2Vec2_zero;
    b2Vec2 f11 = isValid(x + 1, y + 1) ? flow[x + 1][y + 1] : b2Vec2_zero;
    
    //Do the x interpolations
    float32 xWeight = agent.getPosition().x - floor.x;
    
    b2Vec2 top = f00 * (1 - xWeight) +  f10 * xWeight;
    b2Vec2 bottom = f01 * (1 - xWeight) + f11 * xWeight;
    
    //Do the y interpolation
    float32 yWeight = agent.getPosition().y - floor.y;
    
    //This is now the direction we want to be travelling in (needs to be normalized)
    b2Vec2 desiredDirection = top * (1 - yWeight) + bottom * yWeight;
    desiredDirection.Normalize();
    
    //If we are centered on a grid square with no vector this will happen
    if (isnan(desiredDirection.LengthSquared())) {
        return b2Vec2_zero;
    }
    
    return steerTowards(agent, desiredDirection);
}

b2Vec2 MapController::steeringBehaviourSeek(Agent &agent, b2Vec2 dest)
{
    if (dest.x == agent.getPosition().x && dest.y == agent.getPosition().y) {
        return b2Vec2_zero;
    }
    
    b2Vec2 desired = dest - agent.getPosition();
   
    desired *= (agent.maxSpeed / desired.Length());
   
    b2Vec2 velocityChange = desired - agent.getVelocity();
    
    return velocityChange * (agent.maxForce / agent.maxSpeed);
}

b2Vec2 MapController::steeringBehaviourSeparation(Agent &agent)
{
    b2Vec2 totalForce = b2Vec2_zero;
    
    int neighboursCount = 0;
    
    for (int i = 0; i < agents.size(); i++) {
        Agent &a = agents[i];
        if (&a != &agent) {
            float32 distance = B2Vec2DHelper::distanceTo(agent.getPosition(), a.getPosition());
            if (distance < agent.neighbourRadius && distance > 0) {
                b2Vec2 pushForce = agent.getPosition() - a.getPosition();
                float32 length = pushForce.Normalize(); //Normalize returns the original length
                float32 r = (agent.radius + a.radius);
                
                totalForce += pushForce * (1 - length / agent.neighbourRadius);//agent.minSeparation)));
                neighboursCount++;
            }
        }
    }
    
    if (neighboursCount == 0) {
        return totalForce; //Zero
    }
    
    return totalForce * (agent.maxForce / neighboursCount);
}

b2Vec2 MapController::steeringBehaviourCohesion(Agent &agent)
{
    b2Vec2 centerOfMass = b2Vec2_zero;//agent.position().Copy();
    int neighboursCount = 0;
    
    for (int i = 0; i < agents.size(); i++) {
        Agent &a = agents[i];
        if (&a != &agent) {
            float32 distance = B2Vec2DHelper::distanceTo(agent.getPosition(), a.getPosition());
            if (distance < agent.neighbourRadius) {
                //sum up the position of our neighbours
                centerOfMass += a.body->GetPosition();
                neighboursCount++;
            }
        }
    }
    
    if (neighboursCount == 0) {
        return b2Vec2_zero;
    }
    
    //Get the average position of ourself and our neighbours
    centerOfMass *= (1 / neighboursCount);
    
    //seek that position
    return steeringBehaviourSeek(agent, centerOfMass);
}

b2Vec2 MapController::steeringBehaviourAlignment(Agent &agent)
{
    b2Vec2 averageHeading = b2Vec2_zero;
    int neighboursCount = 0;
    
    //for each of our neighbours (including ourself)
    for (int i = 0; i < agents.size(); i++) {
        Agent &a = agents[i];
        float32 distance = B2Vec2DHelper::distanceTo(agent.getPosition(), a.getPosition());
        //That are within the max distance and are moving
        if (distance < agent.neighbourRadius && a.getVelocity().Length() > 0) {
            //Sum up our headings
            b2Vec2 head = a.getVelocity();
            head.Normalize();
            averageHeading += head;
            neighboursCount++;
        }
    }
    
    if (neighboursCount == 0) {
        return averageHeading; //Zero
    }
    
    //Divide to get the average heading
    averageHeading *= (1 / neighboursCount);
    
    //Steer towards that heading
    return steerTowards(agent, averageHeading);
}

b2Vec2 MapController::steerTowards(Agent &agent, b2Vec2 direction)
{
    b2Vec2 desiredVelocity = direction * agent.maxSpeed;
    
    //The velocity change we want
    b2Vec2 velocityChange = desiredVelocity - agent.getVelocity();
    //Convert to a force
    return velocityChange * (agent.maxForce / agent.maxSpeed);
}

void MapController::ccGenerateDijkstraField()
{
    for (int x = 0; x < m_iWidth; x++) {
        for (int y = 0; y < m_iHeight; y++) {
            dijkstra[x][y] = -1;
            lost[x][y] = false;
        }
    }
    
    //Set all places where obstacles are as being weight MAXINT, which will stand for not able to go here
    for (int i = 0; i < obstacles.size(); i++) {
        b2Vec2& t = obstacles[i];
        dijkstra[int(t.x)][int(t.y)] = FLT_MAX;
    }
    
    //flood fill out from the end point
    CostNode pathEnd;
    pathEnd.cost = 0;
    pathEnd.point.x = destinationPoints[0].x;
    pathEnd.point.y = destinationPoints[0].y;
    
    dijkstra[int(pathEnd.point.x)][int(pathEnd.point.y)] = 0;
    lost[int(pathEnd.point.x)][int(pathEnd.point.y)] = true;
    
    std::vector<CostNode> toVisit;
    toVisit.push_back(pathEnd);
    
    //for each node we need to visit, starting with the pathEnd
    for (int i = 0; i < toVisit.size(); i++) {
        CostNode at = toVisit[i];
        
        //cout << "at.x" << at.point.x << "at.y" << at.point.y << "cost" << at.cost << endl;
        
        //Calculate if we have LOS
        //Only need to see if don't have LOS if we aren't the end
        calculateLost(at, pathEnd);
        
        std::vector<CostNode> neighbours = directNeighbours(at);
        
        //for each neighbour of this node (only straight line neighbours, not diagonals)
        for (int j = 0; j < neighbours.size(); j++) {
            CostNode n = neighbours[j];
            
            //We will only ever visit every node once as we are always visiting nodes in the most efficient order
            if (dijkstra[int(n.point.x)][int(n.point.y)] == -1) {
                n.cost = at.cost + 1;
                dijkstra[int(n.point.x)][int(n.point.y)] = n.cost;
                toVisit.push_back(n);
            }
        }
    }
}

void MapController::calculateLost(CostNode at, CostNode pathEnd)
{
    float32 xDif = pathEnd.point.x - at.point.x;
    float32 yDif = pathEnd.point.y - at.point.y;
    
    float32 xDifAbs = fabs(xDif);
    float32 yDifAbs = fabs(yDif);
    
    bool hasLos = false;
    
    int xDifOne = MathLib::sign(xDif);
    int yDifOne = MathLib::sign(yDif);
    
    //Check the direction we are furtherest from the destination on (or both if equal)
    // If it has LOS then we might
    
    //Check in the x direction
    if (xDifAbs >= yDifAbs) {
        
        if (lost[int(at.point.x + xDifOne)][int(at.point.y)]) {
            hasLos = true;
        }
    }
    //Check in the y direction
    if (yDifAbs >= xDifAbs) {
        
        if (lost[int(at.point.x)][int(at.point.y + yDifOne)]) {
            hasLos = true;
        }
    }
    
    //If we are not a straight line vertically/horizontally to the exit
    if (yDifAbs > 0 && xDifAbs > 0) {
        //If the diagonal doesn't have LOS, we don't
        if (!lost[int(at.point.x + xDifOne)][int(at.point.y + yDifOne)]) {
            hasLos = false;
        } else if (yDifAbs == xDifAbs) {
            //If we are an exact diagonal and either straight direction is a wall, we don't have LOS
            if (dijkstra[int(at.point.x + xDifOne)][int(at.point.y)] == FLT_MAX ||
                dijkstra[int(at.point.x)][int(at.point.y + yDifOne)] == FLT_MAX) {
                hasLos = false;
            }
        }
    }
    //It's a definite now
    lost[int(at.point.x)][int(at.point.y)] = hasLos;
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
            if (dijkstra[i][j] == FLT_MAX) continue;
            
            if (lost[i][j]) {
                b2Vec2 p = destinationPoints[0];
                
                p.x -= i;
                p.y -= j;
                p.Normalize();
                
                flow[i][j] = p;
                continue;
            }
            
            b2Vec2 pos(i, j);
            
            std::vector<b2Vec2> neighbours = allNeighbours(pos);
            
            bool isMinFound = false;
            b2Vec2 min;
            float32 minDist = FLT_MAX;
            
            for (int i = 0; i < neighbours.size(); ++i) {
                b2Vec2 n = neighbours[i];
                float32 dist = dijkstra[int(n.x)][int(n.y)] - dijkstra[int(pos.x)][int(pos.y)];
        
                if (dist < minDist) {
                    min = n;
                    minDist = dist;
                    
                    isMinFound = true;
                }
            }
            
            if (isMinFound) {
                b2Vec2 v = min - pos;
                v.Normalize();
                flow[i][j] = v;
            }
        }
    }
}

void MapController::updateDestinationPoint(b2Vec2 newDest)
{
    destinationPoints[0].x = newDest.x;
    destinationPoints[0].y = newDest.y;
}

bool MapController::isValid(int x, int y)
{
    return x >=0 && x < m_iWidth && y >= 0 && y < m_iHeight && dijkstra[x][y] != FLT_MAX;
}

std::vector<b2Vec2> MapController::allNeighbours(b2Vec2 &v)
{
    std::vector<b2Vec2> res;
    int x = v.x;
    int y = v.y;
    
    int up = isValid(x, y - 1);
    int down = isValid(x, y + 1);
    int left = isValid(x - 1, y);
    int right = isValid(x + 1, y);
    
    //We test each straight direction, then subtest the next one clockwise
    
    if (left) {
        res.push_back(b2Vec2(x - 1, y));
        
        //left up
        if (up && isValid(x - 1, y - 1)) {
            res.push_back(b2Vec2(x - 1, y - 1));
        }
    }
    
    if (up) {
        res.push_back(b2Vec2(x, y - 1));
        
        //up right
        if (right && isValid(x + 1, y - 1)) {
            res.push_back(b2Vec2(x + 1, y - 1));
        }
    }
    
    if (right) {
        res.push_back(b2Vec2(x + 1, y));
        
        //right down
        if (down && isValid(x + 1, y + 1)) {
            res.push_back(b2Vec2(x + 1, y + 1));
        }
    }
    
    if (down) {
        res.push_back(b2Vec2(x, y + 1));
        
        //down left
        if (left && isValid(x - 1, y + 1)) {
            res.push_back(b2Vec2(x - 1, y + 1));
        }
    }
    
    return res;
}

std::vector<CostNode> MapController::directNeighbours(CostNode &v)
{
    std::vector<CostNode> ret;
    if (v.point.x > 0) {
        CostNode node;
        node.point.x = v.point.x - 1;
        node.point.y = v.point.y;
        ret.push_back(node);
    }
    if (v.point.y > 0) {
        CostNode node;
        node.point.x = v.point.x;
        node.point.y = v.point.y - 1;
        ret.push_back(node);
    }
    
    if (v.point.x < m_iWidth - 1) {
        CostNode node;
        node.point.x = v.point.x + 1;
        node.point.y = v.point.y;
        ret.push_back(node);
    }
    if (v.point.y < m_iHeight - 1) {
        CostNode node;
        node.point.x = v.point.x;
        node.point.y = v.point.y + 1;
        ret.push_back(node);
    }
    
    return ret;
}