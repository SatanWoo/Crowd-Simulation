//
//  KDTree.h
//  Crowd Simulation
//
//  Created by z on 15/5/26.
//  Copyright (c) 2015年 SatanWoo. All rights reserved.
//

#ifndef Crowd_Simulation_KDTree_h
#define Crowd_Simulation_KDTree_h

#include "KDNode.h"
#include <vector>
#include <deque>

struct KDRange {
    double min;
    double max;
    
    KDRange(double min, double max):min(min),max(max)
    {
    }
    
    bool inRange(double dis)
    {
        return min <= dis && dis <= max;
    }
};

struct KDTree {
    int dimension;
    KDNode *root;
    
    KDTree(const std::vector<KDTuple> &points, int k);
    ~KDTree();
    
    void searchKNearestNeighbours(const KDTuple& goal, std::vector<KDTuple>& neighbours, KDRange range);
private:
    double distance(const KDTuple& a, const KDTuple& b);
    double findMiddleValue(const std::vector<double> values);
    void buildKDTree(KDNode *root, const std::vector<KDTuple> &points, KDSplitAxis axis);
    void search(const KDTuple& goal, std::vector<KDTuple>& neighbours, KDRange range, std::deque<KDNode *> searchPath, bool isBack);
};


#endif
