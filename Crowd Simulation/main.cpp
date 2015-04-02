//
//  main.cpp
//  Crowd Simulation
//
//  Created by z on 15-3-26.
//  Copyright (c) 2015年 SatanWoo. All rights reserved.
//

#include <iostream>
#include <GLUT/GLUT.h>
#include "Vector.h"
#include "MapController.h"

const int MapWidth = 800;
const int MapHeight = 600;

MapController *mapController = NULL;

void glRender()
{
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
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
    glutInit(&argc, const_cast<char**>(argv));
    
    mapController = new MapController(MapWidth, MapHeight, 1000);
    
    glutInitDisplayMode(GLUT_RGBA | GLUT_DOUBLE);
    glutInitWindowSize(MapWidth, MapHeight);
    glutCreateWindow("Crowd Simulation");
    
    glutDisplayFunc(glRender);
    glutIdleFunc(glIdle);
    
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    glLoadIdentity();
    glEnable(GL_POINT_SMOOTH);
    
   
    
    glutMainLoop();

    delete mapController;
    return 0;
}
