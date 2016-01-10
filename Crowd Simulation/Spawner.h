//
//  Spawner.h
//  Crowd Simulation
//
//  Created by z on 16/1/5.
//  Copyright © 2016年 SatanWoo. All rights reserved.
//

#ifndef Spawner_h
#define Spawner_h

#include


RVO::Vector2 getRandSpawn() {
    int temp, temp2, x ,y;
    temp = rand() % 4;
    if(temp == 0)
    {
        x = 125;
        temp2 = rand()%4;
        if(temp2 == 0)
        {
            y = 125;
        }
        else if(temp2 == 1)
        {
            y = 50;
        }
        else if(temp2 == 2)
        {
            y = -50;
        }
        else if(temp2 == 3)
        {
            y = -125;
        }
    }
    else if( temp == 1 )
    {
        x = -125;
        temp2 = rand()%4;
        if(temp2 == 0)
        {
            y = 125;
        }
        else if(temp2 == 1)
        {
            y = 50;
        }
        else if(temp2 == 2)
        {
            y = -50;
        }
        else if(temp2 == 3)
        {
            y = -125;
        }
    }
    else if( temp == 2 )
    {
        x = 50;
        temp2 = rand()%2;
        y = (temp2) ? 125 : -125;
    }
    else if( temp == 3 )
    {
        x = -50;
        temp2 = rand()%2;
        y = (temp2) ? 125 : -125;
    }
    
    return RVO::Vector2(x,y);
}


#endif /* Spawner_h */
