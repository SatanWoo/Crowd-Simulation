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

#ifndef KD_tree_H
#define KD_tree_H

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
    vector<double> min_point_;
    vector<double> max_point_;
    Box()
    {}
    
    Box(const Point& minP, const Point& maxP)
    {
        min_point_.push_back(minP.x);
        min_point_.push_back(minP.y);
        
        max_point_.push_back(maxP.x);
        max_point_.push_back(maxP.y);
    }
    
    Box(const vector<double> & min_point, const vector<double> & max_point)
    {
        min_point_ = min_point;
        max_point_ = max_point;
    }
    
    double distancesq(const Point& p)const
    {
        vector<double> vec;
        vec.push_back(p.x);
        vec.push_back(p.y);
        
        return distancesq(vec);
    }
    
    // squared distance from a point to box
    double distancesq(const vector<double> & point) const
    {
        double sum_sq = 0;
        unsigned long dim= point.size();
        
        for(unsigned int i=0; i<dim; ++i)
        {
            double x_min = min_point_[i];
            double x_max = max_point_[i];
            double x = point[i];
            if(x < x_min)
            {
                sum_sq +=(x_min -x) * (x_min-x);
            }
            else if(x > x_max)
            {
                sum_sq +=(x_max-x) * (x_max -x);
            }
            else
            {
                sum_sq =0;
            }
        }
        return sum_sq;
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

class KD_tree
{
public:
    KD_tree();
    ~KD_tree();

    bool create_tree(const vector<Point>& data, unsigned int max_leaf_size);
    bool kNN_query(const Point& query_point, const int K, vector<int>& indices, vector<double>& squared_distances) const;
    
    // squared of distance
    static double distancesq(const Point& data0, const Point& data1);
private:
    // build and store data
    vector<Point> points;
    size_t dim_;
    unsigned int maxLeafSize;
    unsigned int maxLevel;
    KDNode *root;
    
    // helper function for creating the tree using point indices
    bool buildTree(const vector<int>& index, KDNode*& node, unsigned int level);
    
    // query update from current node
    bool query(const Point & query_point, const int K,
               stack<KDNode *>& nodes,
               set<KDNode *>& visited_nodes,
               priority_queue<DistanceIndex>& priority_points,
               double& max_distance) const;
    
    // check points in a left node
    bool explore_leaf_node(const Point& query_point, const int K,
                           priority_queue<DistanceIndex> & priority_points,
                           double & max_distance, const KDNode * cur_node) const;
    
    Box buildBox(const vector<int> & indices) const;
    
    void recursiveClear(KDNode *node);
};

KD_tree::KD_tree()
{
    root = NULL;
}

KD_tree::~KD_tree()
{
    recursiveClear(root);
}

bool KD_tree::create_tree(const vector<Point>& data, unsigned int max_leaf_size)
{
    points = data;
    maxLeafSize = max_leaf_size;
    maxLevel = ceil(log2(data.size()));
    dim_ = Point::DIMENSION;
    
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
bool KD_tree::buildTree(const vector<int> & indices, KDNode* & node, unsigned level)
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

bool KD_tree::kNN_query(const Point& query_point, const int K,
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
        
        if(value <split_value && cur_node->left!=NULL)
        {
            cur_node = cur_node->left;
        }
        else if(cur_node->right!=NULL)
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

bool KD_tree::query(const Point& query_point, const int K,
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
            this->explore_leaf_node(query_point, K, priority_points, max_distance, cur_node);
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

bool KD_tree::explore_leaf_node(const Point& query_point, const int K,
                                priority_queue<DistanceIndex>& priority_points,
                                double & max_distance, const KDNode* cur_node) const
{
    for(int i = 0; i < cur_node->dataIndex.size(); ++i)
    {
        int index = cur_node->dataIndex[i];
        const Point& point = points[index];
        double dist_sq = KD_tree::distancesq(point, query_point);
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

Box KD_tree::buildBox(const vector<int>& indices) const
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
double KD_tree::distancesq(const Point& data0, const Point& data1)
{
    double diffX = data1.x - data0.x, diffY = data1.y - data1.y;
    
    return diffX * diffX + diffY * diffY;
}

void KD_tree::recursiveClear(KDNode *node)
{
    if (node == NULL) return;
    recursiveClear(node->left);
    recursiveClear(node->right);
    
    delete node;
}


#endif /* KD_TREE_H */
