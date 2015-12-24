#include "MapController.h"
#include "MathLib.h"
#include "Agent.h"
#include "B2Vec2DHelper.h"
#include "KDTuple.h"

#include <GLUT/GLUT.h>
#include <math.h>
#include <deque>
#include <queue>

using namespace std;

const double MapController::restDensity = 1.0;
const double MapController::MapGridSize = 32;

static int frame = 0;

static int fourDir[4][2] = {{0, -1}, {1, 0}, {0, 1}, {-1, 0}};
static int eightDir[8][2] = {{0, -1}, {1, 0}, {0, 1}, {-1, 0}, {1, 1}, {1, -1}, {-1, -1}, {-1, 1}};

MapController::MapController(int width, int height, int count, double timeStep)
{
    tree = new RVOTree(5);
    
	m_iWidth = width;
	m_iHeight = height;
	m_dTimeStep = timeStep;
    
    world = new b2World(b2Vec2_zero, true);

    srand((unsigned)time(NULL));
    
    b2Vec2 v1(m_iWidth - 2, m_iHeight / 2);
    b2Vec2 v2(1, m_iHeight / 2);
    
    destinationPoints.push_back(v1);
    destinationPoints.push_back(v2);

    for (int yPos = 1; yPos < m_iHeight - 1; yPos++) {
        
        for (int i =0 ; i < 3; i++) {
            Agent *p1 = new Agent(b2Vec2(i % 3, yPos), 0);
            p1->initBodyDef(world);
            agents.push_back(p1);
        }
    }
    
    for (int yPos = 1; yPos < m_iHeight - 1; yPos++) {
        
        for (int i =0 ; i < 3; i++) {
            Agent *p1 = new Agent(b2Vec2(m_iWidth - (i + 1) % 3, yPos), 1);
            p1->initBodyDef(world);
            agents.push_back(p1);
        }
    }
    
    for (int i = 0; i < m_iHeight; i++) {
        if (i >= m_iHeight / 2 - 5 && i < m_iHeight / 2 + 5) {
            continue;
        }
        for (int y = 6; y < m_iWidth - 6; y++) {
            obstacles.push_back(b2Vec2(y, i));
        }
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
    
    avgVelocityField = initializeVecField();
    
    densityField = initializeFloatField();
    potentialField = initializeFloatField();
    discomfortField = initializeFloatField();
    
    visited = new bool*[width];
    costField = new FourGrid*[width];
    speedField = new FourGrid*[width];
	for (int i = 0; i < width; i++)
    {
        visited[i] = new bool [height];
        costField[i] = new FourGrid[height];
        speedField[i] = new FourGrid[height];
	}
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
        delete [] visited[i];
        delete [] costField[i];
        delete [] speedField[i];
        
        visited[i] = NULL;
        costField[i] = NULL;
        speedField[i] = NULL;
	}

    delete [] visited;
    visited = NULL;
    
    delete [] costField;
    costField = NULL;
    
    delete [] speedField;
    speedField = NULL;
    
    if (tree)
    {
        delete tree;
        tree = NULL;
    }
    
    for (int i = 0; i < agents.size(); ++i) {
        delete agents[i];
        agents[i] = NULL;
    }
}

#pragma mark - Protected
void MapController::buildKDTree()
{
    if (tree != NULL)
    {
        delete tree;
        tree = NULL;
    }
    
    
    
    //tree = new RVOTree(5);
    
    //tree->buildAgentTree(agents);
}

void MapController::computerNearestNeighbours(double radius)
{
//    for (int i = 0; i < agents.size(); i++) {
//        agents[i].computeNeighbors();
//    }
//}
}

void MapController::mergeNode()
{
//    virtualNodes.clear();
//    for (int i = 0; i < coreNode.size(); i++)
//    {
//        Agent &ai = agents[coreNode[i]];
//        VirtualNode node;
//        
//        node.group = ai.group;
//        node.allNodes.push_back(&ai);
//        
//        for (int k = 0; k < ai.neighbours.size(); k++)
//        {
//            node.allNodes.push_back(&agents[ai.neighbours[k]]);
//        }
//        
//        node.build();
//        virtualNodes.push_back(node);
//    }
}

#pragma mark - End Of Protected
void MapController::render()
{
    glPointSize(10);
    glBegin(GL_POINTS);
    glColor4f(0.0, 1.0, 0.0, 1.0);
    glVertex2d(destinationPoints[0].x * MapGridSize + 0.5 * MapGridSize, destinationPoints[0].y * MapGridSize + 0.5 * MapGridSize);
    glColor4f(0.0, 1.0, 1.0, 1.0);
    glVertex2d(destinationPoints[1].x * MapGridSize + 0.5 * MapGridSize, destinationPoints[1].y * MapGridSize + 0.5 * MapGridSize);
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
                
                if (isValid(kx, ky)) {
                    glVertex2d(xPos , yPos);
                    glVertex2d(kx * MapGridSize, ky * MapGridSize);
                }
            }
        }
    }
    glEnd();
    
    glBegin(GL_QUADS);
    glColor3f(1.0f, 0.0f, 0.0f);
    
    for (int i = 0 ; i < obstacles.size(); i++) {
        b2Vec2 o = obstacles[i];
        int x = o.x;
        int y = o.y;
        
        glVertex2d(x * MapGridSize, y * MapGridSize);
        glVertex2d((x + 1) * MapGridSize, y * MapGridSize);
        glVertex2d((x + 1) * MapGridSize, (y + 1) * MapGridSize);
        glVertex2d(x * MapGridSize, (y + 1) * MapGridSize);
    }
    glEnd();

    
    glPointSize(5);
    glBegin(GL_POINTS);
    for (int i = 0; i < agents.size(); i++) {
        
        Agent *node = agents[i];
        if (node->group == 0) {
            glColor3f(1.0f, 0.0f, 1.0f);
        } else {
            glColor3f(1.0f, 1.0f, 1.0f);
        }
        glVertex2f(node->getPosition().x * MapGridSize + 0.5 * MapGridSize, node->getPosition().y * MapGridSize + 0.5 * MapGridSize);
    }
    glEnd();
}

void MapController::update()
{
    if (frame % 30 == 0)
    {
        buildKDTree();
        computerNearestNeighbours(Agent::radius * 10);
        mergeNode();
    }
    
    frame ++;

    updateContinuumCrowdData();

    int size = agents.size();
    for (int i = size - 1; i >= 0; i--) {
        Agent *node = agents[i];
        
        b2Vec2 ff = node->ff;
        
        b2Vec2 sep = steeringBehaviourSeparation(node);
        b2Vec2 alg = steeringBehaviourAlignment(node);
        b2Vec2 coh = steeringBehaviourCohesion(node);
    
        node->force = ff + sep * 1.2 + alg * 0.3 + coh * 0.05;
        
        float32 lengthSquared =  node->force.LengthSquared();
        if (lengthSquared > Agent::maxForceSquared) {
            node->force *= (Agent::maxForce / sqrt(lengthSquared));
        }
    }
    
    //Move agents based on forces being applied (aka physics)
    for (int i = size - 1; i >= 0; i--) {
        Agent *node = agents[i];
        node->body->ApplyLinearImpulse(node->force * m_dTimeStep, node->getPosition());
//        node.center = node.center + node.force * m_dTimeStep;
//        node.dispatch(m_dTimeStep);
    }
    
    world->Step(m_dTimeStep, 10, 10);
    world->ClearForces();
}

b2Vec2 MapController::steeringBehaviourFlowField(Agent *node)
{
    b2Vec2 floor = B2Vec2DHelper::floorV(node->getPosition());
    
    int x = floor.x;
    int y = floor.y;
    
    b2Vec2 f00 = isValid(x, y) ? flow[x][y] : b2Vec2_zero;
    b2Vec2 f01 = isValid(x, y + 1) ? flow[x][y + 1] : b2Vec2_zero;
    b2Vec2 f10 = isValid(x + 1, y) ? flow[x + 1][y] : b2Vec2_zero;
    b2Vec2 f11 = isValid(x + 1, y + 1) ? flow[x + 1][y + 1] : b2Vec2_zero;
    
    //Do the x interpolations
    float32 xWeight = node->getPosition().x - floor.x;
    
    b2Vec2 top = f00 * (1 - xWeight) +  f10 * xWeight;
    b2Vec2 bottom = f01 * (1 - xWeight) + f11 * xWeight;
    
    //Do the y interpolation
    float32 yWeight = node->getPosition().y - floor.y;
    
    //This is now the direction we want to be travelling in (needs to be normalized)
    b2Vec2 desiredDirection = top * (1 - yWeight) + bottom * yWeight;
    desiredDirection.Normalize();
    
    //If we are centered on a grid square with no vector this will happen
    if (isnan(desiredDirection.LengthSquared())) {
        return b2Vec2_zero;
    }
    
    return steerTowards(node, desiredDirection);
}

b2Vec2 MapController::steeringBehaviourSeek(Agent *node, b2Vec2 dest)
{
    if (dest.x == node->pos.x && dest.y == node->pos.y) {
        return b2Vec2_zero;
    }
    
    b2Vec2 desired = dest - node->pos;
   
    desired *= (Agent::maxSpeed / desired.Length());
   
    b2Vec2 velocityChange = desired - node->getVelocity();
    
    return velocityChange * (Agent::maxForce / Agent::maxSpeed);
}

b2Vec2 MapController::steeringBehaviourSeparation(Agent *node)
{
    b2Vec2 totalForce = b2Vec2_zero;
    
    int neighboursCount = 0;
    
    for (int i = 0; i < agents.size(); i++) {
        Agent *a = agents[i];
        if (&a != &node) {
            float32 distance = B2Vec2DHelper::distanceTo(node->getPosition(), a->getPosition());
            if (distance < Agent::minSeparation && distance > 0) {
                b2Vec2 pushForce = node->getPosition() - a->getPosition();
                float32 length = pushForce.Normalize(); //Normalize returns the original length
                float32 r = (Agent::radius + Agent::radius);
                
                totalForce += pushForce * (1 - ((length - r) / (Agent::minSeparation - r)));//agent.minSeparation)));
                neighboursCount++;
            }
        }
    }
    
    if (neighboursCount == 0) {
        return totalForce; //Zero
    }
    
    return totalForce * (Agent::maxForce / neighboursCount);
}

b2Vec2 MapController::steeringBehaviourCohesion(Agent *node)
{
    b2Vec2 centerOfMass = b2Vec2_zero;//agent.position().Copy();
    int neighboursCount = 0;
    
    for (int i = 0; i < agents.size(); i++) {
        Agent *a = agents[i];
        if (a != node && a->group == node->group) {
            float32 distance = B2Vec2DHelper::distanceTo(node->getPosition(), node->getPosition());
            if (distance < Agent::maxCohesion) {
                //sum up the position of our neighbours
                centerOfMass += a->getPosition();
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
    return steeringBehaviourSeek(node, centerOfMass);
}

b2Vec2 MapController::steeringBehaviourAlignment(Agent *node)
{
    b2Vec2 averageHeading = b2Vec2_zero;
    int neighboursCount = 0;
    
    //for each of our neighbours (including ourself)
    for (int i = 0; i < agents.size(); i++) {
        Agent *a = agents[i];
        float32 distance = B2Vec2DHelper::distanceTo(node->getPosition(), a->getPosition());
        //That are within the max distance and are moving
        if (distance < Agent::maxCohesion && a->getVelocity().Length() > 0 && a->group == node->group) {
            //Sum up our headings
            b2Vec2 head = a->getVelocity();
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
    return steerTowards(node, averageHeading);
}

b2Vec2 MapController::steerTowards(Agent *node, b2Vec2 direction)
{
    b2Vec2 desiredVelocity = direction * Agent::maxSpeed;
    
    //The velocity change we want
    b2Vec2 velocityChange = desiredVelocity - node->getVelocity();
    //Convert to a force
    return velocityChange * (Agent::maxForce / Agent::maxSpeed);
}

// Continnum Crowd
void MapController::updateContinuumCrowdData()
{
    ccClearBuffers();
    
    //Update density field and average speed map (4.1)
    ccCalculateDensityAndAverageSpeed();
    
    //CC Paper says this is group dependant, but I'm not sure how...
    ccCalculateUnitCostField();
    
    for (int group = 0; group <= 1; group++) //foreach group
    {
        ccClearPotentialField();
        
        //Construct the potential
        ccPotentialFieldEikonalFill(destinationPoints[group]);
        //Compute the gradient
        //ccCalculatePotentialFieldGradient();
        ccGenerateFlowField(); //TODO: This does not use the way of calculating described in the paper (I think)
        
        //(use these for steering later)
        for (int i = agents.size() - 1; i >= 0; i--) {
            if (agents[i]->group == group) {
                agents[i]->ff = steeringBehaviourFlowField(agents[i]);
            }
        }
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
    
    for (int i = obstacles.size() - 1; i >= 0; i--) {
        b2Vec2 o = obstacles[i];
        int x = o.x, y = o.y;
        discomfortField[x][y] = INT_MAX;
    }
}

void MapController::ccCalculateDensityAndAverageSpeed()
{
    float32 perAgentDensity = 1.0;
    
    for (int i = 0; i < agents.size(); i++)
    {
        Agent *pi = agents[i];
        
        b2Vec2 floor = B2Vec2DHelper::floorV(pi->getPosition());
        float32 xWeight = pi->getPosition().x - floor.x;
        float32 yWeight = pi->getPosition().y - floor.y;
        
        //top left
        if (isValid(floor.x, floor.y)) {
            ccAddDensity(floor.x, floor.y, pi->getVelocity(), perAgentDensity * (1 - xWeight) * (1 - yWeight));
        }
        //top right
        if (isValid(floor.x + 1, floor.y)) {
            ccAddDensity(floor.x + 1, floor.y, pi->getVelocity(), perAgentDensity * (xWeight) * (1 - yWeight));
        }
        //bottom left
        if (isValid(floor.x, floor.y + 1)) {
            ccAddDensity(floor.x, floor.y + 1, pi->getVelocity(), perAgentDensity * (1 - xWeight) * (yWeight));
        }
        //bottom right
        if (isValid(floor.x + 1, floor.y + 1)) {
            ccAddDensity(floor.x + 1, floor.y + 1, pi->getVelocity(), perAgentDensity * (xWeight) * (yWeight));
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
                
                if (!isValid(targetX, targetY)) {
                    speedField[i][j].value[dir] = FLT_MAX;
                    continue;
                }
                
                float32 veloX = fourDir[dir][0] * avgVelocityField[targetX][targetY].x;
                float32 veloY = fourDir[dir][1] * avgVelocityField[targetX][targetY].y;
                
                //Get the only speed value as one will be zero
                // this is like speedVecX != 0 ? : speedVecX : speedVecY
                float32 flowSpeed = fourDir[dir][0] != 0 ? veloX : veloY;
                
                float32 density = densityField[targetX][targetY];
                
                //cout << "X " << targetX << " Y " << targetY << " " << densityField[targetX][targetY] << endl;
                
                float32 discomfort = discomfortField[targetX][targetY];
                
                if (density >= densityMax) {
                    speedField[i][j].value[dir] = flowSpeed;
                } else if (density <= densityMin) {
                    speedField[i][j].value[dir] = Agent::maxSpeed;
                } else {
                    //medium speed
                    speedField[i][j].value[dir] = Agent::maxSpeed - (density - densityMin) / (densityMax - densityMin) * (4 - flowSpeed);
                }
                
                //we're going to divide by speed later, so make sure it's not zero
                float32 speed = speedField[i][j].value[dir];
                float32 threshold = 0.001;
                speedField[i][j].value[dir] = max(threshold, speed);
                
               
                
                //Work out the cost to move in to the destination cell
                costField[i][j].value[dir] = (speedField[i][j].value[dir] * lengthWeight + timeWeight + discomfortWeight * discomfort) / speedField[i][j].value[dir];
                
                
                //cout << "X:" << i << "Y:" << j << "I:" << dir << " " <<costField[i][j].value[dir] << endl;
            }
        }
    }
}

void MapController::ccClearPotentialField()
{
    for (int i = 0; i < m_iWidth; i++) {
        for (int j = 0; j < m_iHeight; j++) {
            potentialField[i][j] = FLT_MAX;
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
            if (potentialField[i][j] == FLT_MAX) continue;
            
            bool isMinFound = false;
            b2Vec2 min;
            float32 minDist = FLT_MAX;
            
            for (int d = 0; d < 8; d++) {
                if (isValid(i + eightDir[d][0], j + eightDir[d][1])) {
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
    desNode.point = des;
    
    priority_queue<CostNode> pQueue;
    pQueue.push(desNode);
    
    while (!pQueue.empty()) {
        candidatesCount++;
        CostNode at = pQueue.top();
        pQueue.pop();
        
        int x = at.point.x;
        int y = at.point.y;
        
        if (potentialField[x][y] >= at.cost && !visited[x][y]) {
            potentialField[x][y] = at.cost;
            visited[x][y] = true;
            
            for (int i = 0; i < 4; i++) {
                int toX = x + fourDir[i][0];
                int toY = y + fourDir[i][1];
                if (isValid(toX, toY)) {
                    //Cost to go from our target cell to the start
                    //Our cost + cost of moving from the target to us
                    float32 toCost = at.cost + costField[toX][toY].value[(i + 2) % 4];
                    
                    //If we present a better path, overwrite the cost and queue up going to that cell
                    if (toCost < potentialField[toX][toY]) {
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

void MapController::updateDestinationPoint(b2Vec2 newDest)
{
    destinationPoints[0].x = newDest.x;
    destinationPoints[0].y = newDest.y;
    
    destinationPoints[1].x = m_iWidth - destinationPoints[0].x - 1;
    destinationPoints[1].y = m_iHeight - destinationPoints[0].y - 1;
}

bool MapController::isValid(int x, int y)
{
    return x >=0 && x < m_iWidth && y >= 0 && y < m_iHeight;
}