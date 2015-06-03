//
//  KDNode.h
//  Crowd Simulation
//
//  Created by z on 15/5/21.
//  Copyright (c) 2015年 SatanWoo. All rights reserved.
//

#ifndef Crowd_Simulation_KDNode_h
#define Crowd_Simulation_KDNode_h

#include "KDTuple.h"
#include <vector>

typedef enum {
    KDSplitAxisX = 0,
    KDSplitAxisY = 1
}KDSplitAxis;

struct KDNode {
    
    KDNode(KDSplitAxis axis):splitAxis(axis), left(NULL), right(NULL), parent(NULL)
    {
        
    }
    
    size_t _begin;
    size_t _end;
    size_t _left;
    size_t _right;
    
    double _maxX;
    double _minX;
    
    double _maxY;
    double _minY;
    
    KDNode *left;
    KDNode *right;
    KDNode *parent;
    
    std::vector<KDTuple> points;
    KDSplitAxis splitAxis;
    
    bool isLeaf()const{return left == NULL && right == NULL && !points.empty();}
};


#endif
