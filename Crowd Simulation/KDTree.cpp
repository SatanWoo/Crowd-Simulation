//
//  KDTree.cpp
//  Crowd Simulation
//
//  Created by z on 15/5/26.
//  Copyright (c) 2015年 SatanWoo. All rights reserved.
//

#include <stdio.h>
#include "KDTree.h"

KDTree::KDTree(const  std::vector<KDTuple> &points, int k)
{
    this->dimension = k;
    this->root = new KDNode();
    this->buildKDTree(this->root, points, KDSplitAxisX);
}

KDTree::~KDTree()
{
    delete root;
    root = NULL;
}

void KDTree::searchKNearestNeighbours(const KDTuple &goal, std::vector<KDTuple> &neighbours, KDRange range)
{
    if (root == NULL) return;
    
    if (root->isLeaf()) {
        
    }
}

void KDTree::buildKDTree(KDNode *root, const std::vector<KDTuple> &points, KDSplitAxis axis)
{
    if (root == NULL) return;
    if (points.size() == 1)
    {
        root->points.push_back(points[0]);
        return;
    }
    
    std::vector<double> splitValues;
    for (size_t i = 0; i < points.size(); i++)
    {
        splitValues.push_back(points[i][axis]);
    }
    std::sort(splitValues.begin(), splitValues.end());
    
    double middle = findMiddleValue(splitValues);
    root->left = new KDNode();
    root->right = new KDNode();
    
    std::vector<KDTuple> leftPoints;
    std::vector<KDTuple> rightPoints;
    for (size_t i = 0; i < points.size(); i++)
    {
        double val = points[i][axis];
        if (val < middle)
        {
            leftPoints.push_back(points[i]);
        }
        else if (val > middle)
        {
            rightPoints.push_back(points[i]);
        }
        else
        {
             root->points.push_back(points[i]);
        }
    }

    KDSplitAxis next = (KDSplitAxis)(1 - axis);
    buildKDTree(root->left, leftPoints, next);
    buildKDTree(root->right, rightPoints, next);
}

double KDTree::findMiddleValue(const std::vector<double> values)
{
    size_t size = values.size();
    return values[size/2];
}
