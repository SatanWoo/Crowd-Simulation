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

const double ScreenWidth = 800;
const double ScreenHeight = 448;
const double MapWidth = 25;
const double MapHeight = 14;

MapController *mapController = NULL;

//camera values
double xpos = 0, ypos = 0, zpos = 0, xrot = 0, yrot = 0, angle=0.0;
int lastx, lasty;
double xrotrad, yrotrad;

bool shouldPause = false;

#pragma mark - Private
void camera()
{
    glRotatef(xrot, 1.0, 0.0, 0.0);  //rotate our camera on teh x-axis (left and right)
    glRotatef(yrot, 0.0, 1.0, 0.0);  //rotate our camera on the y-axis (up and down)
    glTranslated(-xpos, -ypos, -zpos); //translate the screen to the position of our camera
}

void drawFloor()
{
    glDisable(GL_LIGHTING);
    glColor3f(0, 0.6f, 0.6f);
    glBegin(GL_QUADS);
    glVertex3f(1500.0f, 0, 1500.0f);
    glVertex3f(1500.0f, 0, -1500.0f);
    glVertex3f(-1500.0f, 0, -1500.0f);
    glVertex3f(-1500.0f, 0, 1500.0f);
    glEnd();
    glDisable(GL_LIGHTING);
}

#pragma mark - OpenGL Callback
void glRender()
{
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT | GL_STENCIL_BUFFER_BIT);
    glLoadIdentity();
    glClearColor(0.5f, 0.5f, 0.5f, 1.0f);
    gluLookAt(800, 400, 800.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0);
    
    camera(); // update camera position
    drawFloor();
    mapController->render();
    glutSwapBuffers();
}

void glIdle()
{
    int n = 1;
    while (n--)
    {
        mapController->update();
    }
    
    glutPostRedisplay();
}

void glMotion(int x, int y)
{
//    int diffx = x - (int)lastx; //check the difference between the current x and the last x position
//    int diffy = y - (int)lasty; //check the difference between the current y and the last y position
//    lastx = x;                  //set lastx to the current x position
//    lasty = y;                  //set lasty to the current y position
//    xrot += (float)diffy;       //set the xrot to xrot with the addition of the difference in the y position
//    yrot += (float)diffx;       //set the xrot to yrot with the addition of the difference in the x position
}

void glMouse(int button, int state, int x, int y)
{
    switch (button)
    {
        case GLUT_LEFT_BUTTON:
            int xPos = x / MapController::MapGridSize;
            int yPos = MapHeight -  y / MapController::MapGridSize;
            
            mapController->updateDestinationPoint(b2Vec2(xPos, yPos));
            break;
    }
}

void glKeyboard (unsigned char key, int /*x*/, int /*y*/)
{
    switch (key)
    {
        case 27:
            exit (1);
            break;
        case 'q':
            exit (1);
            break;
        case 'p':
            shouldPause = shouldPause ? 0 : 1;
            break;
        case 'w':
            yrotrad = (yrot / 180 * 3.141592654f);
            xrotrad = (xrot / 180 * 3.141592654f);
            xpos += float(sin(yrotrad)) ;
            zpos -= float(cos(yrotrad)) ;
            ypos -= float(sin(xrotrad)) ;
            break;
        case 'a':
            yrotrad = (yrot / 180 * 3.141592654f);
            xpos -= float(cos(yrotrad)) * 0.2f;
            zpos -= float(sin(yrotrad)) * 0.2f;
            break;
        case 's':
            yrotrad = (yrot / 180 * 3.141592654f);
            xrotrad = (xrot / 180 * 3.141592654f);
            xpos -= float(sin(yrotrad));
            zpos += float(cos(yrotrad));
            ypos += float(sin(xrotrad));
            break;
        case 'd':
            yrotrad = (yrot / 180 * 3.141592654f);
            xpos += float(cos(yrotrad)) * 0.2f;
            zpos += float(sin(yrotrad)) * 0.2f;
            break;
        default:
            break;
    }
}

#pragma mark - OpenGL Init
void glCustomInit()
{
    // LIGHTING
    GLfloat lightpos[4] = { 1.0, 0.0, 1.0, 1.0 };     // light position
    GLfloat lightamb[4] = { 0.0, 0.0, 0.0, 1.0 };     // ambient colour
    GLfloat lightdif[4] = { 1.0, 1.0, 1.0, 1.0 };     // diffuse colour
    GLfloat global_ambient[4] = {0.2f, 0.2f, 0.2f, 1.0f};
    
    glLightfv(GL_LIGHT0, GL_POSITION, lightpos);
    // set the ambient light colour
    glLightfv(GL_LIGHT0, GL_AMBIENT, lightamb);
    // set the diffuse light colour
    glLightfv(GL_LIGHT0, GL_DIFFUSE, lightdif);
    // global ambient
    glLightModelfv(GL_LIGHT_MODEL_AMBIENT, global_ambient);
    // turn on lighting
    glEnable(GL_LIGHTING);
    // enable light 0, all the other lights are off
    glEnable(GL_LIGHT0);
    glEnable(GL_DEPTH_TEST);
    
    glMatrixMode(GL_PROJECTION);
    gluPerspective(60.0, 800./600., 1.0, 2500.0);
    glMatrixMode(GL_MODELVIEW);
    
    glEnable (GL_COLOR_MATERIAL);
    glColorMaterial(GL_FRONT_AND_BACK,GL_AMBIENT_AND_DIFFUSE);
}

int main(int argc, const char * argv[])
{
    glutInit(&argc, const_cast<char **>(argv));
    glutInitDisplayMode(GLUT_RGBA|GLUT_DOUBLE);
    glutInitWindowSize(ScreenWidth, ScreenHeight);
    glutCreateWindow("Crowd Simulation");
    
    glCustomInit();
    
    glutSetCursor(GLUT_CURSOR_NONE);
    
    glutDisplayFunc(glRender);
    glutIdleFunc(glIdle);
    glutMouseFunc(glMouse);
    glutPassiveMotionFunc(glMotion);
    glutKeyboardFunc(glKeyboard);
    
    mapController = new MapController(MapWidth, MapHeight, MapHeight * 2);
    
    glutMainLoop();
    
    delete mapController;
    mapController = NULL;
    return EXIT_SUCCESS;
}
