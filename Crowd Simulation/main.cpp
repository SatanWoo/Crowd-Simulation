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

int main(int argc, const char * argv[])
{
    IrrlichtDevice *device =
    createDevice( video::EDT_SOFTWARE, dimension2d<u32>(ScreenWidth, ScreenHeight), 16,
                 false, false, false, 0);
    
    if (!device)
        return 1;
    
    IVideoDriver* driver = device->getVideoDriver();
    ISceneManager* smgr = device->getSceneManager();
    IGUIEnvironment* guienv = device->getGUIEnvironment();
        
    /*
     To look at the mesh, we place a camera into 3d space at the position
     (0, 30, -40). The camera looks from there to (0,5,0), which is
     approximately the place where our md2 model is.
     */
    smgr->addCameraSceneNode(0, vector3df(0,30,-40), vector3df(0,5,0));
    
    MapController *mapController = new MapController(MapWidth, MapHeight, MapHeight * 2);
    mapController->setDevice(device);
    
    while(device->run())
    {
        driver->beginScene(true, true, SColor(255,100,101,140));
        
        mapController->update();
        
        smgr->drawAll();
        guienv->drawAll();
        
        driver->endScene();
    }
    
    delete mapController;
    mapController = NULL;
    device->drop();
    
    return 0;
}
