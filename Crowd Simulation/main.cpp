//
//  main.cpp
//  Crowd Simulation
//
//  Created by z on 15-3-26.
//  Copyright (c) 2015年 SatanWoo. All rights reserved.
//


#include <GLUT/GLUT.h>
#include <iostream>
#include <fstream>
#include "Vector.h"
#include "MapController.h"

using namespace std;

const double MapWidth = 25;
const double MapHeight = 14;

ofstream fout("/Users/z/Documents/Crowd\ Simulation/Crowd\ Simulation/SkipFrame-FPS.txt");

const double ScreenWidth = MapWidth * MapController::MapGridSize;
const double ScreenHeight = MapHeight * MapController::MapGridSize;

MapController *mapController = NULL;

bool stop = true;

int now = 0, start = 0, frame = 0;

void glRender()
{
    //if (stop) return;
    
    frame++;
    glClear( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT );
    glLoadIdentity();
    
    mapController->render();
    glutSwapBuffers();
    
    if (frame % 100 == 0)
    {
        cout << "frame is %d" << frame << endl;
        stop = true;
    }
}

void glIdle()
{
    int n = 1;
    while (n--) {
        mapController->update();
    }
    
    now = glutGet(GLUT_ELAPSED_TIME);
    
    if (now - start > 1000)
    {
        double fps = (double)(frame * 1000.0) / (double)(now - start);
        fout << mapController->totalAgents() * 10 << "  " << fps << endl;
    }
    
    glutPostRedisplay();
}

void glMouse(int button, int state, int x, int y)
{
    switch (button)
    {
        case GLUT_LEFT_BUTTON:
            int xPos = x / MapController::MapGridSize;
            int yPos = MapHeight -  y / MapController::MapGridSize;
            
            cout << xPos << ":" << yPos << endl;
            
            mapController->updateDestinationPoint(b2Vec2(xPos, yPos));
            break;
    }
}

void glKeyboard(unsigned char key, int x, int y)
{
    switch (key) {
        case 'b':
            mapController->switchState();
            break;
    }
}

int main(int argc, const char * argv[])
{
    glutInit(&argc, const_cast<char **>(argv));
    glutInitDisplayMode(GLUT_RGBA|GLUT_DOUBLE);
    glutInitWindowSize(ScreenWidth, ScreenHeight);
    glutCreateWindow("Crowd Simulation");
    
    glutDisplayFunc(glRender);
    glutIdleFunc(glIdle);
    glutMouseFunc(glMouse);
    glutKeyboardFunc(glKeyboard);
    
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluOrtho2D(0, MapWidth * MapController::MapGridSize, 0, MapHeight * MapController::MapGridSize);
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    glEnable(GL_POINT_SMOOTH);
    //glEnable(GL_BLEND);
    
    mapController = new MapController(MapWidth, MapHeight, MapHeight * 2);

    glutMainLoop();
    
    fout.close();

    delete mapController;
    mapController = NULL;
    return EXIT_SUCCESS;
}
