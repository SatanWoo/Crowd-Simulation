//
//  main.cpp
//  Crowd Simulation
//
//  Created by z on 15-3-26.
//  Copyright (c) 2015年 SatanWoo. All rights reserved.
//


#include <GLUT/GLUT.h>
#include <iostream>
#include "irrlicht.h"
#include "Vector.h"
#include "MapController.h"

#define RESOURCEFOLDER @"/Users/z/Documents/Crowd Simulation/media"

using namespace std;
using namespace irr;
using namespace core;
using namespace scene;
using namespace video;
using namespace io;
using namespace gui;

const double ScreenWidth = 800;
const double ScreenHeight = 448;
const double MapWidth = 25;
const double MapHeight = 14;

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
    int n = 1;
    while (n--) {
        mapController->update();
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

int main(int argc, const char * argv[])
{
//    glutInit(&argc, const_cast<char **>(argv));
//    glutInitDisplayMode(GLUT_RGBA|GLUT_DOUBLE);
//    glutInitWindowSize(ScreenWidth, ScreenHeight);
//    glutCreateWindow("Crowd Simulation");
//    
//    glutDisplayFunc(glRender);
//    glutIdleFunc(glIdle);
//    glutMouseFunc(glMouse);
//    
//    glMatrixMode(GL_PROJECTION);
//    glLoadIdentity();
//    gluOrtho2D(0, MapWidth * MapController::MapGridSize, 0, MapHeight * MapController::MapGridSize);
//    glMatrixMode(GL_MODELVIEW);
//    glLoadIdentity();
//    glEnable(GL_POINT_SMOOTH);
//    //glEnable(GL_BLEND);
//    
//    mapController = new MapController(MapWidth, MapHeight, MapHeight * 2);
//
//    glutMainLoop();
//
//    delete mapController;
//    mapController = NULL;
//    return EXIT_SUCCESS;
    
    IrrlichtDevice *device =
    createDevice( video::EDT_SOFTWARE, dimension2d<u32>(640, 480), 16,
                 false, false, false, 0);
    
    if (!device)
        return 1;
    
    /*
     Set the caption of the window to some nice text. Note that there is an
     'L' in front of the string. The Irrlicht Engine uses wide character
     strings when displaying text.
     */
    device->setWindowCaption(L"Hello World! - Irrlicht Engine Demo");
    
    /*
     Get a pointer to the VideoDriver, the SceneManager and the graphical
     user interface environment, so that we do not always have to write
     device->getVideoDriver(), device->getSceneManager(), or
     device->getGUIEnvironment().
     */
    IVideoDriver* driver = device->getVideoDriver();
    ISceneManager* smgr = device->getSceneManager();
    IGUIEnvironment* guienv = device->getGUIEnvironment();
    
    /*
     We add a hello world label to the window, using the GUI environment.
     The text is placed at the position (10,10) as top left corner and
     (260,22) as lower right corner.
     */
    guienv->addStaticText(L"Hello World! This is the Irrlicht Software renderer!",
                          rect<s32>(10,10,260,22), true);
    
    /*
     To show something interesting, we load a Quake 2 model and display it.
     We only have to get the Mesh from the Scene Manager with getMesh() and add
     a SceneNode to display the mesh with addAnimatedMeshSceneNode(). We
     check the return value of getMesh() to become aware of loading problems
     and other errors.
     
     Instead of writing the filename sydney.md2, it would also be possible
     to load a Maya object file (.obj), a complete Quake3 map (.bsp) or any
     other supported file format. By the way, that cool Quake 2 model
     called sydney was modelled by Brian Collins.
     */
    IAnimatedMesh* mesh = smgr->getMesh("/Users/z/Documents/Crowd Simulation/media/sydney.md2");
    if (!mesh)
    {
        device->drop();
        return 1;
    }
    IAnimatedMeshSceneNode* node = smgr->addAnimatedMeshSceneNode( mesh );
    
    /*
     To let the mesh look a little bit nicer, we change its material. We
     disable lighting because we do not have a dynamic light in here, and
     the mesh would be totally black otherwise. Then we set the frame loop,
     such that the predefined STAND animation is used. And last, we apply a
     texture to the mesh. Without it the mesh would be drawn using only a
     color.
     */
    if (node)
    {
        node->setMaterialFlag(EMF_LIGHTING, false);
        node->setMD2Animation(scene::EMAT_STAND);
        node->setMaterialTexture( 0, driver->getTexture("/Users/z/Documents/Crowd Simulation/media/sydney.bmp") );
    }
    
    /*
     To look at the mesh, we place a camera into 3d space at the position
     (0, 30, -40). The camera looks from there to (0,5,0), which is
     approximately the place where our md2 model is.
     */
    smgr->addCameraSceneNode(0, vector3df(0,30,-40), vector3df(0,5,0));
    
    /*
     Ok, now we have set up the scene, lets draw everything: We run the
     device in a while() loop, until the device does not want to run any
     more. This would be when the user closes the window or presses ALT+F4
     (or whatever keycode closes a window).
     */
    while(device->run())
    {
        /*
         Anything can be drawn between a beginScene() and an endScene()
         call. The beginScene() call clears the screen with a color and
         the depth buffer, if desired. Then we let the Scene Manager and
         the GUI Environment draw their content. With the endScene()
         call everything is presented on the screen.
         */
        driver->beginScene(true, true, SColor(255,100,101,140));
        
        smgr->drawAll();
        guienv->drawAll();
        
        driver->endScene();
    }
    
    /*
     After we are done with the render loop, we have to delete the Irrlicht
     Device created before with createDevice(). In the Irrlicht Engine, you
     have to delete all objects you created with a method or function which
     starts with 'create'. The object is simply deleted by calling ->drop().
     See the documentation at irr::IReferenceCounted::drop() for more
     information.
     */
    device->drop();
    
    return 0;
}
