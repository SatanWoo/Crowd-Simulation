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
#include "MapController.h"

using namespace std;

const int ScreenWidth = 800;
const int ScreenHeight = 600;
const int MapWidth = 60;
const int MapHeight = 60;

MapController *mapController = NULL;

void glRender()
{
    glClear( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT );
    glLoadIdentity();
    
    mapController->render();
    glutSwapBuffers();
}

void glIdle()
{
    mapController->update();
    glutPostRedisplay();
}

int main(int argc, const char * argv[])
{
    glutInit(&argc, const_cast<char **>(argv));
    glutInitDisplayMode(GLUT_RGBA|GLUT_DOUBLE);
    glutInitWindowSize(ScreenWidth, ScreenHeight);
    glutCreateWindow("Crowd Simulation");
    
    glutDisplayFunc(glRender);
    glutIdleFunc(glIdle);
    
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluOrtho2D(0, MapWidth, 0, MapHeight);
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    glEnable(GL_POINT_SMOOTH);
    
    mapController = new MapController(MapWidth, MapHeight, 10);

    glutMainLoop();

    delete mapController;
    mapController = NULL;
    return EXIT_SUCCESS;
}
