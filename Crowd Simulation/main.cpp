//
//  main.cpp
//  Crowd Simulation
//
//  Created by z on 15-3-26.
//  Copyright (c) 2015年 SatanWoo. All rights reserved.
//


#include <GLUT/GLUT.h>
#include <iostream>
#include "Vector.h"
#include "SimulationController.h"
#include "MapController.h"

using namespace std;

const double ScreenWidth = 800;
const double ScreenHeight = 448;
const double MapWidth = 30;
const double MapHeight = 30;

//const double ScreenWidth = MapWidth * MapController::MapGridSize;
//const double ScreenHeight = MapHeight * MapController::MapGridSize;

SimulationController *controller = NULL;

void glRender()
{
    glClear( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT );
    glLoadIdentity();
    
    controller->render();
    glutSwapBuffers();
}

void glIdle()
{
    int n = 1;
    while (n--) {
        controller->simulate();
    }
    
    glutPostRedisplay();
}

void glMouse(int button, int state, int x, int y)
{
    switch (button)
    {
//        case GLUT_LEFT_BUTTON:
//            int xPos = x / MapController::MapGridSize;
//            int yPos = MapHeight -  y / MapController::MapGridSize;
//            
//            cout << xPos << ":" << yPos << endl;
//            
//            mapController->updateDestinationPoint(b2Vec2(xPos, yPos));
//            break;
    }
}

int main(int argc, const char * argv[])
{
    glutInit(&argc, const_cast<char **>(argv));
    glutInitDisplayMode(GLUT_RGBA|GLUT_DOUBLE);
    glutInitWindowSize(ScreenWidth, ScreenHeight);
    glutCreateWindow("Cluster Simulation");
    
    glutDisplayFunc(glRender);
    glutIdleFunc(glIdle);
    glutMouseFunc(glMouse);
    
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluOrtho2D(0, MapWidth * MapController::MapGridSize, 0, MapHeight * MapController::MapGridSize);
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    glEnable(GL_POINT_SMOOTH);
    //glEnable(GL_BLEND);
    
    controller = new SimulationController(MapHeight * 2);
    controller->init(MapWidth, MapHeight);
    controller->setRelation(b2Vec3(1.2, 0.3, 0.05));
    controller->setTimeStep(0.02);
    
    glutMainLoop();

    delete controller;
    controller = NULL;
    return EXIT_SUCCESS;
}
