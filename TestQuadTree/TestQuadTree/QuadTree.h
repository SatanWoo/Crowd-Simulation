//
//  QuadTree.h
//  TestQuadTree
//
//  Created by z on 15/12/13.
//  Copyright © 2015年 SatanWoo. All rights reserved.
//

#ifndef QuadTree_h
#define QuadTree_h

#include <vector>
#include <stack>
#include <queue>
#include <algorithm>
#include <set>
using namespace std;

typedef enum {
    QuadAreaNW = 0,
    QuadAreaNE = 1,
    QuadAreaSW = 2,
    QuadAreaSE = 3
} QuadArea;

struct Point
{
    double x, y;
    Point(double cx = 0, double cy = 0): x(cx), y(cy)
    {}
    
    double distanceSquare(const Point& p2)
    {
        double diffX = p2.x - x, diffY = p2.y - y;
        return diffX * diffX + diffY * diffY;
    }
};

struct Box
{
    double xMin, xMax;
    double yMin, yMax;
    
    Box():xMin(0), xMax(0), yMin(0), yMax(0)
    {}
    
    Box(double x0, double x1, double y0, double y1):xMin(x0), xMax(x1), yMin(y0), yMax(y1)
    {}
    
    Box(const Box& b)
    {
        xMin = b.xMin, xMax = b.xMax;
        yMin = b.yMin, yMax = b.yMax;
    }
    
    bool isBoxIntersect(const Box& b2)
    {
        if (this == &b2) return true;
        
        bool isNotIntersect = b2.yMax > yMax || b2.yMin < yMin || b2.xMin < xMin || b2.xMax > xMax;
        return !isNotIntersect;
    }
    
    bool containBox(const Box& b2)
    {
        return b2.xMin >= xMin && b2.xMax <= xMax && b2.yMin >= yMin && b2.yMax <= yMax;
    }
    
    bool containPoint(const Point& p)
    {
        return xMin <= p.x && p.x <= xMax && yMin <= p.y && p.y <= yMax;
    }
    
    // squared distance from a point to box
    double distancesq(const Point& p) const
    {
        double sumX, sumY = 0;
        
        if(p.x < xMin)
        {
            sumX = (xMin - p.x) * (xMin - p.x);
        }
        else if(p.x > xMax)
        {
            sumX = (p.x - xMax) * (p.x - xMax);
        }
        else
        {
            sumY = 0;
        }
        
        if(p.y < yMin)
        {
            sumY = (yMin - p.y) * (yMin - p.y);
        }
        else if(p.x > xMax)
        {
            sumY = (p.y - yMax) * (p.y - yMax);
        }
        else
        {
            sumY = 0;
        }
        
        return sumX + sumY;
    }
};

static Box findMaxBox(const vector<Point>& data)
{
    Point minP, maxP;
    for (int i = 0; i < data.size(); ++i) {
        const Point &p = data[i];
        if (p.x < minP.x)
        {
            minP.x = p.x;
        }
        else if (p.x > maxP.x)
        {
            maxP.x = p.x;
        }
        
        if (p.y < minP.y)
        {
            minP.y = p.y;
        }
        else if (p.y > maxP.y)
        {
            maxP.y = p.y;
        }
    }
    
    return Box(minP.x, maxP.x, minP.y, maxP.y);
}

class DistanceIndex
{
public:
    int index;
    double distance;
    
    DistanceIndex(int index, double distance)
    {
        this->index = index;
        this->distance = distance;
    }
    
    bool operator < (const DistanceIndex & other) const
    {
        return this->distance < other.distance;
    }
    
    bool operator > (const DistanceIndex & other) const
    {
        return this->distance > other.distance;
    }
};

struct QuadNode {
    struct QuadNode *northwest;
    struct QuadNode *northeast;
    struct QuadNode *southwest;
    struct QuadNode *southeast;
    
    Box box;
    int leafSize;
    int count;
    vector<int> indices;
    
    QuadNode(const Box& b, int leaf):northwest(NULL), northeast(NULL), southwest(NULL), southeast(NULL), box(b), count(0), leafSize(leaf)
    {
        indices.resize(leafSize);
    }
    
    QuadNode *candidateChildNode(const Point& p)
    {
        if (isNotDivided()) return this;
        
        double xMed = (box.xMin + box.xMax) / 2.0;
        double yMed = (box.yMin + box.xMax) / 2.0;
        
        if (p.x >= xMed && p.y >= yMed)
        {
            return southeast;
        }
        else if (p.x >= xMed && p.y <= yMed)
        {
            return northeast;
        }
        else if (p.x <= xMed && p.y >= yMed)
        {
            return southwest;
        }
        else
        {
            return northwest;
        }
    }
    
    bool isNotDivided()const
    {
        return northwest == NULL;
    }
};

class QuadTree
{
public:
    QuadTree(const vector<Point>& dataset, unsigned int leafSize);
    virtual ~QuadTree();
    
    void KNNSearch(const Point& query, const int K, vector<int>& index, vector<double>& distance);
    
private:
    void subdivideNode(QuadNode *node);
    bool insertData(int index, QuadNode *node);
    
    void recursiveClearNode(QuadNode *node);
    void query(const Point & query_point, const int K,
               stack<QuadNode *>& nodes,
               set<QuadNode *>& visited_nodes,
               priority_queue<DistanceIndex>& priority_points,
               double& max_distance) const;
    
    // check points in a non-divied node
    void exploreNonDivideNode(const Point& query_point, const int K,
                         priority_queue<DistanceIndex>& priority_points,
                         double &max_distance, const QuadNode* cur_node) const;
    
    QuadNode *root;
    vector<Point> points;
};

QuadTree::QuadTree(const vector<Point>& data, unsigned int leafSize)
{
    root = NULL;
    points = data;
    
    vector<int> indices;
    for (int i = 0; i < data.size(); ++i)
    {
        indices.push_back(i);
    }
    
    root = new QuadNode(findMaxBox(data), leafSize);
    for (int i = 0; i < data.size(); ++i)
    {
        insertData(i, root);
    }
}

QuadTree::~QuadTree()
{
    recursiveClearNode(root);
}

void QuadTree::KNNSearch(const Point &q, const int K, vector<int> &indices, vector<double> &distances)
{
    if (root == NULL) return;
    
    double max_sq_distance = numeric_limits<double>::max();
    priority_queue<DistanceIndex> distancequeue;
    
    stack<QuadNode *> candidate_nodes;
    QuadNode* cur_node = root;
    
    // DFS
    while(cur_node != NULL)
    {
        candidate_nodes.push(cur_node);
        
        if(cur_node->isNotDivided()) break;
        
        QuadNode *node = cur_node->candidateChildNode(q);
        if (node == cur_node) break;
        
        cur_node = node;
    }
    
    set<QuadNode *> visited_nodes;
    query(q, K, candidate_nodes, visited_nodes, distancequeue, max_sq_distance);
    
    indices.resize(K);
    distances.resize(K);
    
    int num = K -1;
    while(!distancequeue.empty())
    {
        DistanceIndex top = distancequeue.top();
        distancequeue.pop();
        indices[num] = top.index;
        distances[num] = top.distance;
        num--;
    }
}

void QuadTree::query(const Point &query_point, const int K, stack<QuadNode *> &nodes, set<QuadNode *> &visited_nodes, priority_queue<DistanceIndex> &priority_points, double &max_distance)const
{
    while(!nodes.empty())
    {
        QuadNode* cur_node = nodes.top();
        nodes.pop();
        
        //already visited node
        if(visited_nodes.find(cur_node)!= visited_nodes.end())
        {
            continue;
        }
        
        visited_nodes.insert(cur_node);
        if(cur_node->isNotDivided())
        {
            //this->exploreLeafNode(query_point, K, priority_points, max_distance, cur_node);
        }
        else
        {
            double distSQ = cur_node->northeast->box.distancesq(query_point);
            if(distSQ < max_distance)
            {
                nodes.push(cur_node->northeast);
            }
            
            distSQ = cur_node->northwest->box.distancesq(query_point);
            if(distSQ < max_distance)
            {
                nodes.push(cur_node->northwest);
            }
            
            distSQ = cur_node->southeast->box.distancesq(query_point);
            if(distSQ < max_distance)
            {
                nodes.push(cur_node->southeast);
            }
            
            distSQ = cur_node->southwest->box.distancesq(query_point);
            if(distSQ < max_distance)
            {
                nodes.push(cur_node->southwest);
            }
        }
    }
}

void QuadTree::exploreNonDivideNode(const Point &query_point, const int K, priority_queue<DistanceIndex> &priority_points, double &max_distance, const QuadNode *cur_node)const
{
    for(int i = 0; i < cur_node->indices.size(); ++i)
    {
        int index = cur_node->indices[i];
        Point point = points[index];
        double dist_sq = point.distanceSquare(query_point);
        if(priority_points.size() < K || dist_sq < max_distance)
        {
            DistanceIndex di(index, dist_sq);
            priority_points.push(di);
            if(priority_points.size() > K)
            {
                priority_points.pop();
            }
            max_distance = priority_points.top().distance;
        }
    }
}

bool QuadTree::insertData(int index, QuadNode *node)
{
    const Point& p = points[index];
    Box box = node->box;
    
    if (!box.containPoint(p)) return false;
    
    if (node->count < node->leafSize)
    {
        node->indices[node->count++] = index;
        return true;
    }
    
    if (node->isNotDivided())
    {
        subdivideNode(node);
    }
    
    QuadNode *candidateInsertNode = node->candidateChildNode(p);
    
    if (candidateInsertNode == node) return false;
    return insertData(index, candidateInsertNode);
}

void QuadTree::subdivideNode(QuadNode *node)
{
    if (node == NULL) return;
    
    Box box = node->box;
    
    double x = box.xMax + box.xMin, y = box.yMax + box.yMin;
    double medX = x/2.0, medY = y/2.0;
    
    node->northwest = new QuadNode(Box(box.xMin, medX, box.yMin, medY), node->leafSize);
    node->northeast = new QuadNode(Box(medX, box.xMax, box.yMin, medY), node->leafSize);
    node->southeast = new QuadNode(Box(medX, box.xMax, medY, box.yMax), node->leafSize);
    node->southwest = new QuadNode(Box(box.xMin, medX, medY, box.yMax), node->leafSize);
}

void QuadTree::recursiveClearNode(QuadNode *node)
{
    if (node == NULL) return;
    recursiveClearNode(node->southwest);
    recursiveClearNode(node->southeast);
    recursiveClearNode(node->northeast);
    recursiveClearNode(node->northwest);
    
    delete node;
    node = NULL;
}

#endif /* QuadTree_h */
