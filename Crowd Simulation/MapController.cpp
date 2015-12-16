#include "MapController.h"
#include <GLUT/GLUT.h>

using namespace std;

const double MapController::restDensity = 1.0;
const double MapController::MapGridSize = 32;

static int frame = 0;

static int fourDir[4][2] = {{0, -1}, {1, 0}, {0, 1}, {-1, 0}};
static int eightDir[8][2] = {{0, -1}, {1, 0}, {0, 1}, {-1, 0}, {1, 1}, {1, -1}, {-1, -1}, {-1, 1}};

MapController::MapController(int width, int height)
{
    setMapSize(width, height);
}

MapController::~MapController()
{
}

void MapController::render()
{
    glLineWidth(1);
    glBegin(GL_LINES);
    glColor4f(1.0f, 1.0f, 1.0f, 0.1);
    for (int i = 0; i < this->width; i++)
    {
        for (int j = 0; j < this->height; j++)
        {
            double xPos = i * MapGridSize;
            double yPos = j * MapGridSize;
            
            for (int k = 0; k < 4; k++)
            {
                int kx = i + fourDir[k][0];
                int ky = j + fourDir[k][1];
                
                if (isInMap(kx, ky)) {
                    glVertex2d(xPos , yPos);
                    glVertex2d(kx * MapGridSize, ky * MapGridSize);
                }
            }
        }
    }
    glEnd();
    
//    glBegin(GL_QUADS);
//    glColor3f(1.0f, 0.0f, 0.0f);
//    
//    for (int i = 0 ; i < obstacles.size(); i++) {
//        b2Vec2 o = obstacles[i];
//        int x = o.x;
//        int y = o.y;
//        
//        glVertex2d(x * MapGridSize, y * MapGridSize);
//        glVertex2d((x + 1) * MapGridSize, y * MapGridSize);
//        glVertex2d((x + 1) * MapGridSize, (y + 1) * MapGridSize);
//        glVertex2d(x * MapGridSize, (y + 1) * MapGridSize);
//    }
//    glEnd();
}

//void MapController::updateDestinationPoint(b2Vec2 newDest)
//{
//    destinationPoints[0].x = newDest.x;
//    destinationPoints[0].y = newDest.y;
//    
//    destinationPoints[1].x = m_iWidth - destinationPoints[0].x - 1;
//    destinationPoints[1].y = m_iHeight - destinationPoints[0].y - 1;
//}

bool MapController::isInMap(int x, int y)const
{
    return x >=0 && x < this->width && y >= 0 && y < this->height;
}