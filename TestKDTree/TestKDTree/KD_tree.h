/**
 * File: KDTree.h
 * Author Lili Meng (lilimeng1103@gmail.com) based on part of the KD tree code by Jimmy Chen.
 * Thanks a lot for the fruitful discussion with Jimmy Chen, Victor Gan
 * ------------------------
 * Perform constructing trees, efficient exact query for k-nearest neighbors based on Bounded priority queue kd-tree,
 * Best-Bin-First(BBF) query for approximate k-nearest neighbors search.
 * For more BBF query, please refer to
 * Beis, J. S. and Lowe, D. G.  Shape indexing using approximate nearest-neighbor search in high-dimensional spaces.
 *
 * An interface representing a kd-tree in some number of dimensions. The tree
 * can be constructed from a set of data and then queried for membership and
 * nearest neighbors.
 **/

#ifndef KDTree_H
#define KDTree_H

#include <vector>
#include <assert.h>
#include <queue>
#include <stack>
#include <set>
#include <algorithm>
#include <fstream>
#include <iostream>
#include <unordered_map>
#include <stdio.h>
#include <math.h>

using namespace std;

struct Point
{
    static const int DIMENSION = 2;
    double x, y;
    
    Point():x(0), y(0)
    {}
    
    Point(const Point& p)
    {
        x = p.x, y = p.y;
    }
    
    double distanceSquare(const Point& p2)
    {
        double diffX = p2.x - x, diffY = p2.y - y;
        return diffX * diffX + diffY * diffY;
    }
};

class Box
{
public:
    double x0, x1;
    double y0, y1;
    
    Box()
    {}
    
    Box(const Point& minP, const Point& maxP)
    {
        x0 = minP.x; x1 = maxP.x;
        y0 = minP.y; y1 = maxP.y;
    }
    
    Box(double x0, double x1, double y0, double y1):x0(x0), x1(x1), y0(y0), y1(y1)
    {}
    
    
    // squared distance from a point to box
    double distancesq(const Point& p) const
    {
        double sumX, sumY = 0;
        
        if(p.x < x0)
        {
            sumX = (x0 - p.x) * (x0 - p.x);
        }
        else if(p.x > x1)
        {
            sumX = (p.x - x1) * (p.x - x1);
        }
        else
        {
            return 0;
        }
        
        if(p.y < y0)
        {
            sumY = (y0 - p.y) * (y0 - p.y);
        }
        else if(p.x > x1)
        {
            sumY = (p.y - y1) * (p.y - y1);
        }
        else
        {
            return 0;
        }
        
        
        return sumX + sumY;
    }
};

class KDNode
{
public:
    int level;
    int splitDim;
    double splitValue;
    bool isLeaf;
    Box box;
    
    KDNode *left;
    KDNode *right;
    vector<int> dataIndex;
    
    KDNode():level(0), splitDim(0), splitValue(0), isLeaf(false), left(NULL), right(NULL)
    {}
};

/**For storing the KDNode index and the distance from node to node **/
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

class KDTree
{
public:
    KDTree(const vector<Point>& data, unsigned int max_leaf_size);
    ~KDTree();

    
    bool KNNQuery(const Point& query_point, const int K, vector<int>& indices, vector<double>& squared_distances) const;
    
    // squared of distance
    static double distancesq(const Point& data0, const Point& data1);
private:
    // build and store data
    vector<Point> points;
    unsigned int maxLeafSize;
    unsigned int maxLevel;
    KDNode *root;
    
    // helper function for creating the tree using point indices
    bool createTree(const vector<Point>& data, unsigned int max_leaf_size);
    bool buildTree(const vector<int>& index, KDNode*& node, unsigned int level);
    
    // query update from current node
    bool query(const Point & query_point, const int K,
               stack<KDNode *>& nodes,
               set<KDNode *>& visited_nodes,
               priority_queue<DistanceIndex>& priority_points,
               double& max_distance) const;
    
    // check points in a left node
    bool exploreLeafNode(const Point& query_point, const int K,
                           priority_queue<DistanceIndex> & priority_points,
                           double & max_distance, const KDNode * cur_node) const;
    
    Box buildBox(const vector<int> & indices) const;
    
    void recursiveClear(KDNode *node);
};

KDTree::KDTree(const vector<Point>& data, unsigned int max_leaf_size)
{
    root = NULL;
    createTree(data, max_leaf_size);
}

KDTree::~KDTree()
{
    recursiveClear(root);
}

bool KDTree::createTree(const vector<Point>& data, unsigned int max_leaf_size)
{
    points = data;
    maxLeafSize = max_leaf_size;
    maxLevel = ceil(log2(data.size()));
    
    vector<int> indices;
    for(double i = 0; i< points.size(); i++)
    {
        indices.push_back(i);
    }
    
    root = new KDNode();
    root->box=this->buildBox(indices);
    this->buildTree(indices, root, 0);
    
    return true;
}

// helper function for creating the tree using point indices
bool KDTree::buildTree(const vector<int> & indices, KDNode* & node, unsigned level)
{
    if(level >= maxLevel || indices.size() <= maxLeafSize)
    {
        node->isLeaf = true;
        node->level = level;
        node->dataIndex = indices;
        node->box = this->buildBox(indices);
        return true;
    }
    
    node->level = level;
    node->dataIndex = indices;
    
    // randomly select split dimensions
    int split_dim = rand() % Point::DIMENSION;
    node->splitDim = split_dim;
    vector<double> one_dim_values;
    for(int i = 0; i < indices.size(); ++i)
    {
        Point p = points[indices[i]];
        double val = split_dim == 0 ? p.x : p.y;
        one_dim_values.push_back(val);
    }
    
    // median value as split value, every value before the meidan is less than the median, every value after the median is larger than the median;
    size_t n = one_dim_values.size() / 2;
    std::nth_element(one_dim_values.begin(), one_dim_values.begin()+n, one_dim_values.end());
    
    node->splitValue = one_dim_values[n];
    node->box = this->buildBox(indices);
    
    vector<int> left_indices;
    vector<int> right_indices;
    for(int i = 0; i< indices.size(); i++)
    {
        int index = indices[i];
        Point p = points[indices[i]];
        double v = split_dim == 0 ? p.x : p.y;
        if(v < node->splitValue)
        {
            left_indices.push_back(index);
        }
        else
        {
            right_indices.push_back(index);
        }
    }
    
    if(left_indices.size() != 0)
    {
        KDNode* left_node = new KDNode();
        this->buildTree(left_indices, left_node, level+1);
        node->left = left_node;
    }
    
    if(right_indices.size() != 0)
    {
        KDNode *right_node = new KDNode();
        this->buildTree(right_indices, right_node, level+1);
        node->right = right_node;
    }
    
    return true;
}

bool KDTree::KNNQuery(const Point& query_point, const int K,
                        vector<int>& indices,
                        vector<double>& squared_distances) const
{
    double max_sq_distance = numeric_limits<double>::max();
    priority_queue<DistanceIndex> distancequeue;
    
    stack<KDNode*> candidate_nodes;
    KDNode* cur_node = root;
    
    // travel to the leaf node
    while(cur_node!=NULL)
    {
        candidate_nodes.push(cur_node);
        
        if(cur_node->isLeaf)
        {
            break;
        }
        int dim = cur_node->splitDim;
        double split_value = cur_node->splitValue;
        double value = dim == 0 ? query_point.x : query_point.y;
        
        if(value < split_value && cur_node->left != NULL)
        {
            cur_node = cur_node->left;
        }
        else if(cur_node->right != NULL)
        {
            cur_node = cur_node->right;
        }
    }
    
    set<KDNode*> visited_nodes;
    this->query(query_point, K, candidate_nodes, visited_nodes, distancequeue, max_sq_distance);
    
    indices.resize(K);
    squared_distances.resize(K);
    
    int num = K -1;
    while(!distancequeue.empty())
    {
        DistanceIndex top = distancequeue.top();
        distancequeue.pop();
        indices[num] = top.index;
        squared_distances[num] = top.distance;
        num--;
    }
    
    return true;
}

bool KDTree::query(const Point& query_point, const int K,
                    stack<KDNode*>& nodes,
                    set<KDNode*>& visited_nodes,
                    priority_queue<DistanceIndex>& priority_points,
                    double & max_distance) const
{
    while(!nodes.empty())
    {
        KDNode* cur_node = nodes.top();
        nodes.pop();
        
        //already visited node
        if(visited_nodes.find(cur_node)!= visited_nodes.end())
        {
            continue;
        }
        
        visited_nodes.insert(cur_node);
        if(cur_node->isLeaf)
        {
            this->exploreLeafNode(query_point, K, priority_points, max_distance, cur_node);
        }
        else
        {
            // internal node
            if(cur_node->left)
            {
                double dist_sq = cur_node->left->box.distancesq(query_point);
                if(dist_sq < max_distance)
                {
                    nodes.push(cur_node->left);
                }
            }
            if(cur_node->right)
            {
                double dist_sq = cur_node->left->box.distancesq(query_point);
                if(dist_sq < max_distance)
                {
                    nodes.push(cur_node->right);
                }
            }
        }
    }
    return true;
}

bool KDTree::exploreLeafNode(const Point& query_point, const int K,
                                priority_queue<DistanceIndex>& priority_points,
                                double & max_distance, const KDNode* cur_node) const
{
    for(int i = 0; i < cur_node->dataIndex.size(); ++i)
    {
        int index = cur_node->dataIndex[i];
        const Point& point = points[index];
        double dist_sq = KDTree::distancesq(point, query_point);
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
    return true;
}

Box KDTree::buildBox(const vector<int>& indices) const
{
    Point minP= points[indices.front()];
    Point maxP = minP;
    
    for(unsigned int i = 1; i< indices.size(); ++i)
    {
        Point p = points[indices[i]];
        
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
    
    return Box(minP, maxP);
}

//For brute-force comparison
double KDTree::distancesq(const Point& data0, const Point& data1)
{
    double diffX = data1.x - data0.x, diffY = data1.y - data1.y;
    return diffX * diffX + diffY * diffY;
}

void KDTree::recursiveClear(KDNode *node)
{
    if (node == NULL) return;
    recursiveClear(node->left);
    recursiveClear(node->right);
    
    delete node;
}

#endif /* KDTree_H */
