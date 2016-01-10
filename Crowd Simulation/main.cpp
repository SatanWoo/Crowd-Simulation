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
#include "Obstacle.h"

using namespace std;

const double ScreenWidth = 800;
const double ScreenHeight = 600;
const double MapWidth = 25;
const double MapHeight = 14;

MapController *mapController = NULL;
ObstacleContainer container;

//camera values
double xpos = 0, ypos = 0, zpos = 0, xrot = 0, yrot = 0, angle=0.0;
int lastx, lasty;
double xrotrad, yrotrad;

bool shouldPause = false;

std::vector<b2Vec2> intersections;
std::vector<b2Vec2> ob;

#pragma mark - Declaration
void camera();
void drawFloor();
void drawAgent();
void drawBuildings();
void drawObstacles();
void drawWalkways();

// makes a 100x100 intersection centered at x,y
void makeIntersection(double x, double y ) {
    b2Vec2 center(x, y);
    
    intersections.push_back(center);
    ob.clear();
    // center
    ob.push_back(b2Vec2(x+5.0f, y-5.0f));
    ob.push_back(b2Vec2(x+5.0f, y+5.0f));
    ob.push_back(b2Vec2(x-5.0f, y+5.0f));
    ob.push_back(b2Vec2(x-5.0f, y-5.0f));
    
    container.makeObstacle(ob);
    ob.clear();
    
    // bottom
    ob.push_back(b2Vec2(x-5.0f, y+50.0f));
    ob.push_back(b2Vec2(x-5.0f, y+10.0f));
    ob.push_back(b2Vec2(x+5.0f, y+10.0f));
    ob.push_back(b2Vec2(x+5.0f, y+50.0f));
    
    container.makeObstacle(ob);
    ob.clear();
    
    //top
    ob.push_back(b2Vec2(x-5.0f, y-10.0f));
    ob.push_back(b2Vec2(x-5.0f, y-50.0f));
    ob.push_back(b2Vec2(x+5.0f, y-50.0f));
    ob.push_back(b2Vec2(x+5.0f, y-10.0f));
    
    container.makeObstacle(ob);
    ob.clear();
    
    //left
    ob.push_back(b2Vec2(x-50.0f, y+5.0f));
    ob.push_back(b2Vec2(x-50.0f, y-5.0f));
    ob.push_back(b2Vec2(x-10.0f, y-5.0f));
    ob.push_back(b2Vec2(x-10.0f, y+5.0f));
    
    container.makeObstacle(ob);
    ob.clear();
    
    //right
    ob.push_back(b2Vec2(x+10.0f, y+5.0f));
    ob.push_back(b2Vec2(x+10.0f, y-5.0f));
    ob.push_back(b2Vec2(x+50.0f, y-5.0f));
    ob.push_back(b2Vec2(x+50.0f, y+5.0f));
    
    container.makeObstacle(ob);
    ob.clear();
}

#pragma mark - OpenGL Callback
void glRender()
{
//
//    mapController->render();
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT | GL_STENCIL_BUFFER_BIT);
    glLoadIdentity();
    glClearColor(0.5f, 0.5f, 0.5f, 1.0f);
    gluLookAt(10.0, 100.0, 80.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0);
    
    camera(); // update camera position
    
    drawFloor();
    drawBuildings();
    drawObstacles();
    mapController->render();
    
    glutSwapBuffers();
}

void glIdle()
{
    int n = 1;
    while (n--) {
        mapController->update();
    }
    
    glutPostRedisplay();
}

void glMotion(int x, int y) {
    int diffx=x-(int)lastx; //check the difference between the current x and the last x position
    int diffy=y-(int)lasty; //check the difference between the current y and the last y position
    lastx=x; //set lastx to the current x position
    lasty=y; //set lasty to the current y position
    xrot += (float) diffy; //set the xrot to xrot with the addition of the difference in the y position
    yrot += (float) diffx;    //set the xrot to yrot with the addition of the difference in the x position
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
    switch (key) {
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

#pragma mark - Draw Method
void drawFloor()
{
    glDisable(GL_LIGHTING);
    glColor3f(0.6, 0.6f, 0.6f);
    glBegin(GL_QUADS);
    glVertex3f(150.0f, -0.01f, 150.0f);
    glVertex3f(150.0f, -0.01f, -150.0f);
    glVertex3f(-150.0f, -0.01f, -150.0f);
    glVertex3f(-150.0f, -0.01f, 150.0f);
    glEnd();
    glDisable(GL_LIGHTING);
}

void drawBuildings()
{
    glDisable(GL_LIGHTING);
    glColor3f( 0.6f, 0.4f, 0.4f);
    glBegin(GL_QUADS);
    
    // big building 1 - tall
    glVertex3f(10, 0, -10);
    glVertex3f(90, 0, -10);
    glVertex3f(90, 100, -10);
    glVertex3f(10, 100, -10);
    glVertex3f(10, 0, -90);
    glVertex3f(90, 0, -90);
    glVertex3f(90, 100, -90);
    glVertex3f(10, 100, -90);
    glVertex3f(10, 0, -10);
    glVertex3f(10, 0, -90);
    glVertex3f(10, 100, -90);
    glVertex3f(10, 100, -10);
    glVertex3f(90, 0, -10);
    glVertex3f(90, 0, -90);
    glVertex3f(90, 100, -90);
    glVertex3f(90, 100, -10);
    
    //chopped block
    glColor3f(0.8f, 0.0f, 0.0f); // deep red
    glVertex3f(10, 0, 10);
    glVertex3f(30, 0, 10);
    glVertex3f(30, 10, 10);
    glVertex3f(10, 10, 10);
    glVertex3f(10, 0, 30);
    glVertex3f(30, 0, 30);
    glVertex3f(30, 10, 30);
    glVertex3f(10, 10, 30);
    glVertex3f(10, 0, 10);
    glVertex3f(10, 0, 30);
    glVertex3f(10, 10, 30);
    glVertex3f(10, 10, 10);
    glVertex3f(30, 0, 10);
    glVertex3f(30, 0, 30);
    glVertex3f(30, 10, 30);
    glVertex3f(30, 10, 10);
    glVertex3f(10, 10, 10);
    glVertex3f(30, 10, 10);
    glVertex3f(30, 10, 30);
    glVertex3f(10, 10, 30);
    
    glColor3f(0.0f, 0.6f, 0.6f); // teal
    glVertex3f(60, 0, 10);
    glVertex3f(90, 0, 10);
    glVertex3f(90, 30, 10);
    glVertex3f(60, 30, 10);
    glVertex3f(60, 0, 30);
    glVertex3f(90, 0, 30);
    glVertex3f(90, 30, 30);
    glVertex3f(60, 30, 30);
    glVertex3f(60, 0, 10);
    glVertex3f(60, 0, 30);
    glVertex3f(60, 30, 30);
    glVertex3f(60, 30, 10);
    glVertex3f(90, 0, 10);
    glVertex3f(90, 0, 30);
    glVertex3f(90, 30, 30);
    glVertex3f(90, 30, 10);
    glVertex3f(60, 30, 10);
    glVertex3f(90, 30, 10);
    glVertex3f(90, 30, 30);
    glVertex3f(60, 30, 30);
    
    glColor3f(0.8f, 0.75f, 0.0f);
    glVertex3f(80, 0, 35);
    glVertex3f(90, 0, 35);
    glVertex3f(90, 5, 35);
    glVertex3f(80, 5, 35);
    glVertex3f(80, 0, 90);
    glVertex3f(90, 0, 90);
    glVertex3f(90, 5, 90);
    glVertex3f(80, 5, 90);
    glVertex3f(80, 0, 35);
    glVertex3f(80, 0, 90);
    glVertex3f(80, 5, 90);
    glVertex3f(80, 5, 35);
    glVertex3f(90, 0, 35);
    glVertex3f(90, 0, 90);
    glVertex3f(90, 5, 90);
    glVertex3f(90, 5, 35);
    glVertex3f(80, 5, 35);
    glVertex3f(90, 5, 35);
    glVertex3f(90, 5, 90);
    glVertex3f(80, 5, 90);
    
    glColor3f(0.0f, 0.6f, 0.0f);
    glVertex3f(70, 0, 35);
    glVertex3f(75, 0, 35);
    glVertex3f(75, 2, 35);
    glVertex3f(70, 2, 35);
    glVertex3f(70, 0, 90);
    glVertex3f(75, 0, 90);
    glVertex3f(75, 2, 90);
    glVertex3f(70, 2, 90);
    glVertex3f(70, 0, 90);
    glVertex3f(70, 0, 35);
    glVertex3f(70, 2, 35);
    glVertex3f(70, 2, 90);
    glVertex3f(75, 0, 90);
    glVertex3f(75, 0, 35);
    glVertex3f(75, 2, 35);
    glVertex3f(75, 2, 90);
    glVertex3f(70, 2, 35);
    glVertex3f(75, 2, 35);
    glVertex3f(75, 2, 90);
    glVertex3f(70, 2, 90);
    
    glColor3f(0.85f, 0.85f, 0.85f);
    glVertex3f(30, 0, 60);
    glVertex3f(65, 0, 60);
    glVertex3f(65, 15, 60);
    glVertex3f(30, 15, 60);
    glVertex3f(30, 0, 90);
    glVertex3f(65, 0, 90);
    glVertex3f(65, 15, 90);
    glVertex3f(30, 15, 90);
    glVertex3f(30, 0, 60);
    glVertex3f(30, 0, 90);
    glVertex3f(30, 15, 90);
    glVertex3f(30, 15, 60);
    glVertex3f(65, 0, 60);
    glVertex3f(65, 0, 90);
    glVertex3f(65, 15, 90);
    glVertex3f(65, 15, 60);
    glVertex3f(30, 15, 60);
    glVertex3f(65, 15, 60);
    glVertex3f(65, 15, 90);
    glVertex3f(30, 15, 90);
    
    glColor3f(0.0f, 0.0f, 0.85f);
    glVertex3f(10, 0, 60);
    glVertex3f(20, 0, 60);
    glVertex3f(20, 40, 60);
    glVertex3f(10, 40, 60);
    glVertex3f(10, 0, 90);
    glVertex3f(20, 0, 90);
    glVertex3f(20, 40, 90);
    glVertex3f(10, 40, 90);
    glVertex3f(10, 0, 90);
    glVertex3f(10, 0, 60);
    glVertex3f(10, 40, 60);
    glVertex3f(10, 40, 90);
    glVertex3f(20, 0, 90);
    glVertex3f(20, 0, 60);
    glVertex3f(20, 40, 60);
    glVertex3f(20, 40, 90);
    glVertex3f(10, 40, 90);
    glVertex3f(10, 40, 60);
    glVertex3f(20, 40, 60);
    glVertex3f(20, 40, 90);
    
    glColor3f(0.65f, 0.0f, 0.65f);
    glVertex3f(10, 0, 35);
    glVertex3f(30, 0, 35);
    glVertex3f(30, 4, 35);
    glVertex3f(10, 4, 35);
    glVertex3f(10, 0, 55);
    glVertex3f(30, 0, 55);
    glVertex3f(30, 4, 55);
    glVertex3f(10, 4, 55);
    glVertex3f(10, 0, 55);
    glVertex3f(10, 0, 35);
    glVertex3f(10, 4, 35);
    glVertex3f(10, 4, 55);
    glVertex3f(30, 0, 55);
    glVertex3f(30, 0, 35);
    glVertex3f(30, 4, 35);
    glVertex3f(30, 4, 55);
    glVertex3f(10, 4, 35);
    glVertex3f(30, 4, 35);
    glVertex3f(30, 4, 55);
    glVertex3f(10, 4, 55);
    
    glEnd();
    
    glColor3f(0.75, 0.75f, 1.0f);
    glPushMatrix();
    glTranslatef(-50, 20, -50);
    glutSolidSphere(40, 50, 50);
    glPopMatrix();
    glEnable(GL_LIGHTING);
}

void drawWalkways()
{
    for(size_t i = 0; i < intersections.size(); i++){
        glColor3f(1.15f, 0.15f, 0.15f);
        glBegin(GL_QUADS);
        // top
        glVertex3f(intersections[i].x-5.0f, 0.0f, intersections[i].y-5.0f);
        glVertex3f(intersections[i].x-5.0f, 0.0f, intersections[i].y-10.0f);
        glVertex3f(intersections[i].x+5.0f, 0.0f, intersections[i].y-10.0f);
        glVertex3f(intersections[i].x+5.0f, 0.0f, intersections[i].y-5.0f);
        
        // bottom
        glVertex3f(intersections[i].x-5.0f, 0.0f, intersections[i].y+5.0f);
        glVertex3f(intersections[i].x-5.0f, 0.0f, intersections[i].y+10.0f);
        glVertex3f(intersections[i].x+5.0f, 0.0f, intersections[i].y+10.0f);
        glVertex3f(intersections[i].x+5.0f, 0.0f, intersections[i].y+5.0f);
        
        // left
        glVertex3f(intersections[i].x-5.0f, 0.0f, intersections[i].y+5.0f);
        glVertex3f(intersections[i].x-5.0f, 0.0f, intersections[i].y-5.0f);
        glVertex3f(intersections[i].x-10.0f, 0.0f, intersections[i].y-5.0f);
        glVertex3f(intersections[i].x-10.0f, 0.0f, intersections[i].y+5.0f);
        
        // right
        glVertex3f(intersections[i].x+5.0f, 0.0f, intersections[i].y+5.0f);
        glVertex3f(intersections[i].x+5.0f, 0.0f, intersections[i].y-5.0f);
        glVertex3f(intersections[i].x+10.0f, 0.0f, intersections[i].y-5.0f);
        glVertex3f(intersections[i].x+10.0f, 0.0f, intersections[i].y+5.0f);
        
        glEnd();
        
        glLineWidth(3.0f);
        glColor3f(1.0f, 1.0f, 1.0f);
        glBegin(GL_LINES);
        glVertex3f(intersections[i].x-5.0f, 0.05f, intersections[i].y-10.0f);
        glVertex3f(intersections[i].x+5.0f, 0.05f, intersections[i].y-10.0f);
        glVertex3f(intersections[i].x-5.0f, 0.05f, intersections[i].y-5.0f);
        glVertex3f(intersections[i].x+5.0f, 0.05f, intersections[i].y-5.0f);
        glVertex3f(intersections[i].x-5.0f, 0.05f, intersections[i].y+10.0f);
        glVertex3f(intersections[i].x+5.0f, 0.05f, intersections[i].y+10.0f);
        glVertex3f(intersections[i].x-5.0f, 0.05f, intersections[i].y+5.0f);
        glVertex3f(intersections[i].x+5.0f, 0.05f, intersections[i].y+5.0f);
        
        glVertex3f(intersections[i].x-5.0f, 0.05f, intersections[i].y-5.0f);
        glVertex3f(intersections[i].x-5.0f, 0.05f, intersections[i].y+5.0f);
        glVertex3f(intersections[i].x+5.0f, 0.05f, intersections[i].y-5.0f);
        glVertex3f(intersections[i].x+5.0f, 0.05f, intersections[i].y+5.0f);
        glVertex3f(intersections[i].x-10.0f, 0.5f, intersections[i].y-5.0f);
        glVertex3f(intersections[i].x-10.0f, 0.05f, intersections[i].y+5.0f);
        glVertex3f(intersections[i].x+10.0f, 0.05f, intersections[i].y-5.0f);
        glVertex3f(intersections[i].x+10.0f, 0.05f, intersections[i].y+5.0f);
        glEnd();
    }
}

void drawObstacles()
{
    glColor3f( 0.15f, 1.f, 0.15f);
    size_t obstacles = container.obstacles_.size();
    
    glBegin(GL_QUADS);
    for( size_t i =0; i < obstacles-4; i++){
        b2Vec2 vec = container.obstacles_[i]->point_;
        glVertex3f(vec.x, 0.0, vec.y);
    }
    glEnd();
}

#pragma mark - initialize
// swiftless tutorial for camera - direct copy (see top comment )
void camera()
{
    glRotatef(xrot,1.0,0.0,0.0);  //rotate our camera on teh x-axis (left and right)
    glRotatef(yrot,0.0,1.0,0.0);  //rotate our camera on the y-axis (up and down)
    glTranslated(-xpos, -ypos, -zpos); //translate the screen to the position of our camera
}

void initializeScene()
{
    makeIntersection(0.0, 0.0);
    makeIntersection(0.0, 100.0);
    makeIntersection(100.0, 0.0);
    makeIntersection(100.0, 100.0);
    makeIntersection(0.0, -100.0);
    makeIntersection(-100.0, 0.0);
    makeIntersection(-100.0, -100.0);
    makeIntersection(-100.0, 100.0);
    makeIntersection(100.0, -100.0);
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
