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

static int fourDir[4][2] = {{0, -1}, {1, 0}, {0, 1}, {-1, 0}};
static int eightDir[8][2] = {{0, -1}, {1, 0}, {0, 1}, {-1, 0}, {1, 1}, {1, -1}, {-1, -1}, {-1, 1}};

MapController::MapController(int width, int height, int count, double timeStep)
{
	m_iWidth = width;
	m_iHeight = height;
	m_dTimeStep = timeStep;
    
    world = new b2World(b2Vec2_zero, true);

    srand((unsigned)time(NULL));
    
    for (int yPos = 1; yPos < m_iHeight - 1; yPos++) {
        Agent p1(b2Vec2(0, yPos), 0);
        Agent p2(b2Vec2(1, yPos), 0);
        Agent p3(b2Vec2(2, yPos), 0);
        
        agents.push_back(p1);
        agents.push_back(p2);
        agents.push_back(p3);
    }
    
    for (int yPos = 1; yPos < m_iHeight - 1; yPos++) {
        Agent p1(b2Vec2(m_iWidth - 1, yPos), 1);
        Agent p2(b2Vec2(m_iWidth - 2, yPos), 1);
        Agent p3(b2Vec2(m_iWidth - 3, yPos), 1);
        
        agents.push_back(p1);
        agents.push_back(p2);
        agents.push_back(p3);
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

}


//b2Vec2 MapController::steeringTowards(int pID, b2Vec2 desiredDirection)
//{
//    Person &pi = people[pID];
//    b2Vec2 desiredVelocity = desiredDirection * pi.getMaxSpeed();
//    b2Vec2 velocityChange = desiredVelocity - pi.getLinearVelocity();
//    
//    return velocityChange * (pi.getMaxForce() / pi.getMaxSpeed());
//}

void MapController::update()
{
    updateContinuumCrowdData();

    
    world->Step(m_dTimeStep, 10, 10);
    world->ClearForces();
}

bool MapController::isValid(int x, int y)
{
    if (x < 0 || x >= m_iWidth) return false;
    if (y < 0 || y >= m_iHeight) return false;
    return true;
}

void MapController::render()
{
    glPointSize(10);
    glBegin(GL_POINTS);
    glColor4f(0.0, 1.0, 0.0, 1.0);
    //glVertex2d(destinationPoint.x, destinationPoint.y);
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
                    glVertex2d(xPos, yPos);
                    glVertex2d(kx * MapGridSize, ky * MapGridSize);
                }
            }
        }
    }
    glEnd();
   
    glPointSize(5);
    glBegin(GL_POINTS);
    for (int i = 0; i < agents.size(); i++) {
        
        Agent &agent = agents[i];
        if (agent.group == 0) {
            glColor3f(1.0f, 0.0f, 1.0f);
        } else {
            glColor3f(1.0f, 1.0f, 1.0f);
        }
        //p.render();
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
    //ccPotentialFieldEikonalFill(destinationPoint);
    ccGenerateFlowField(); //TODO: This does not use the way of calculating described in the paper (I think)
//    
//    for (int i = m_iCount - 1; i >= 0; i--) {
//        Person &pi = people[i];
//        pi.flowForce = steeringFromFlowFleid(i, destinationPoint);;
//    }
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
    
    for (int i = 0; i < agents.size(); i++)
    {
        Agent &pi = agents[i];
        
        b2Vec2 floor = B2Vec2DHelper::floorV(pi.getPosition());
        float32 xWeight = pi.getPosition().x - floor.x;
        float32 yWeight = pi.getPosition().y - floor.y;
        
        //top left
        if (isValid(floor.x, floor.y)) {
            ccAddDensity(floor.x, floor.y, pi.getVelocity(), perAgentDensity * (1 - xWeight) * (1 - yWeight));
        }
        //top right
        if (isValid(floor.x + 1, floor.y)) {
            ccAddDensity(floor.x + 1, floor.y, pi.getVelocity(), perAgentDensity * (xWeight) * (1 - yWeight));
        }
        //bottom left
        if (isValid(floor.x, floor.y + 1)) {
            ccAddDensity(floor.x, floor.y + 1, pi.getVelocity(), perAgentDensity * (1 - xWeight) * (yWeight));
        }
        //bottom right
        if (isValid(floor.x + 1, floor.y + 1)) {
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
                
                if (!isValid(targetX, targetY)) {
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
    desNode.point.x = des.x / MapGridSize;
    desNode.point.y = des.y / MapGridSize;
    
    priority_queue<CostNode> pQueue;
    pQueue.push(desNode);
    
    //cout << "Width" << m_iWidth << "Height" << m_iHeight << endl;
    
    while (!pQueue.empty()) {
        candidatesCount++;
        CostNode at = pQueue.top();
        pQueue.pop();
        
        int x = at.point.x;
        int y = at.point.y;
        
        //cout << "Point is " << x << "::" << y <<  "::" <<  at.cost << endl;
        
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