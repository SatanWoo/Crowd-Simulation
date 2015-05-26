//
//  KDTree.cpp
//  Crowd Simulation
//
//  Created by z on 15/5/26.
//  Copyright (c) 2015年 SatanWoo. All rights reserved.
//

#include <stdio.h>
#include "KDTree.h"
#include <math.h>

KDTree::KDTree(const std::vector<KDTuple> &points, int k)
{
    this->dimension = k;
    this->root = new KDNode(KDSplitAxisX);
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
    
    std::deque<KDNode *> path;
    path.push_back(root);
    search(goal, neighbours, range, path, false);
}

void KDTree::search(const KDTuple& goal, std::vector<KDTuple>& neighbours, KDRange range, std::deque<KDNode *> searchPath, bool isBack)
{
    KDNode *last = searchPath.back();
    KDTuple p = last->points.front();
    KDRange newRange = range;
    double dis = distance(last->points.front(), goal);
    
    if (last->isLeaf())
    {
        searchPath.pop_back();
    
        
        if (range.inRange(dis))
        {
            newRange.max = dis;
            neighbours.clear();
            neighbours.push_back(last->points.front());
        }
        
        return search(goal, neighbours, newRange, searchPath, true);
    }
    
    if (isBack)
    {
        searchPath.pop_back();
        double plane = p[last->splitAxis];
        double planeDiff = fabs(plane - goal[last->splitAxis]);
        if (range.inRange(planeDiff))
        {
            newRange.max = dis;
        }
        
        if (p[last->splitAxis] < goal[last->splitAxis])
        {
            // Go to Left Child;
            searchPath.push_back(last->left);
        }
        else
        {
            // Go to Right Child
            searchPath.push_back(last->right);
        }
    }
    
    else
    {
        if (p[last->splitAxis] > goal[last->splitAxis])
        {
            // Go to Left Child;
            searchPath.push_back(last->left);
        }
        else
        {
            // Go to Right Child
            searchPath.push_back(last->right);
        }
    }
    
    search(goal, neighbours, newRange, searchPath, isBack);
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
    
    KDSplitAxis next = (KDSplitAxis)(1 - axis);
    
    root->left = new KDNode(next);
    root->left->parent = root;
    root->right = new KDNode(next);
    root->right->parent = root;
    
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
    
    buildKDTree(root->left, leftPoints, next);
    buildKDTree(root->right, rightPoints, next);
}

double KDTree::distance(const KDTuple &a, const KDTuple &b)
{
    double xDiff = a[0] - b[0];
    double yDiff = a[1] - b[1];
    
    return sqrt(xDiff * xDiff + yDiff * yDiff);
}

double KDTree::findMiddleValue(const std::vector<double> values)
{
    size_t size = values.size();
    return values[size/2];
}
