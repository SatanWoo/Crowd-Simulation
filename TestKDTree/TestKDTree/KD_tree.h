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

typedef enum {
    KDTreeSplitAxisX = 0,
    KDTreeSplitAxisY = 1
} KDTreeSplitAxis;

struct Point {
    static const int dimension = 2;
    
    double x, y;
    Point():x(0), y(0)
    {}
    
    double squareDistance(const Point& p2)
    {
        return (p2.x - x) * (p2.x - x) + (p2.y - y) * (p2.y - y);
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
    
    Box(const vector<double> & min_point, const vector<double> & max_point)
    {
        xMin = min_point[0], yMin = min_point[1];
        xMax = min_point[1], yMax = min_point[1];
    }

    // squared distance from a point to box
    double distanceSQ(const Point& point) const
    {
        double sumX = 0, sumY = 0;
        
        if (point.x > xMax) {
            sumX = pow(point.x - xMax, 2);
        } else if (point.x < xMin) {
            sumX = pow(xMin - point.x, 2);
        } else {
            sumX = 0;
        }
        
        if (point.y > yMax) {
            sumY = pow(point.y - yMax, 2);
        } else if (point.y < yMin) {
            sumY = pow(yMin - point.y, 2);
        } else {
            sumY = 0;
        }
        
        return sumX + sumY;
    }
};

struct KDNode
{
    int level;
    KDTreeSplitAxis splitDim;
    double splitValue;
    bool isLeaf;
    Box box;

    KDNode *left;
    KDNode *right;
    vector<int> dataIndex;

    KDNode():level(0), splitDim(KDTreeSplitAxisX), splitValue(0), isLeaf(false), left(NULL), right(NULL)
    {}
};

/**For storing the KDNode index and the distance from node to node **/
struct DistanceIndex
{
    int index;
    double distance;

    DistanceIndex(int index, double distance)
    {
        index = index;
        distance = distance;
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

/**For storing the tree node and the distance from node to node **/
struct TreeNodeDistance
{
    KDNode* node;
    double distance;

    TreeNodeDistance(KDNode* node, double distance)
    {
        this->node = node;
        this->distance = distance;
    }

    bool operator < (const TreeNodeDistance & other) const
    {
        return this->distance < other.distance;
    }

    bool operator > (const TreeNodeDistance & other) const
    {
        return this->distance > other.distance;
    }
};

class KDTree
{
public:
    KDTree(const vector<Point>& data, unsigned int maxLeafSize);
    ~KDTree();
    
    size_t dimension() const;
    size_t size() const;
    bool empty() const;
    bool kNNQuery(const Point& query, const int K, vector<int>& indices, vector<double>& disSq)const;

private:
    typedef priority_queue<TreeNodeDistance, vector<TreeNodeDistance>, greater<TreeNodeDistance>> minQueue;
    // build and store data
    vector<Point> data;
    unsigned maxLeafSize;
    unsigned maxLevel;
    KDNode *root;

    // helper function for creating the tree using point indices
    bool createTree(const vector<Point> & data, unsigned int maxLeafSize);
    bool buildTree(const vector<int> & index, KDNode* & node, unsigned int level);
    const KDNode* getLeafNode(const Point& query_point, const KDNode * node)const;

    // query update from current node
    bool query(const Point& query_point, const int K,
               stack<KDNode *> & nodes,
               set<KDNode *> & visited_nodes,
               priority_queue<DistanceIndex> & priority_points,
               double & max_distance) const;

    // check points in a left node
    bool exploreLeafNode(const Point& query_point, const int K,
                           priority_queue<DistanceIndex> & priority_points,
                           double & max_distance, const KDNode * node)const;

    Box buildBoundingBox(const vector<int>& indices) const;
};

KDTree::KDTree(const vector<Point>& data, unsigned int maxLeafSize)
{
    root = NULL;
    this->createTree(data, maxLeafSize);
}

KDTree::~KDTree()
{
    delete root;
    root = NULL;
}

bool KDTree::empty()const
{
    if (data.size() == 0) return true;
    return false;
}

size_t KDTree::dimension()const
{
    return Point::dimension;
}

size_t KDTree::size()const
{
    return data.size();
}

bool KDTree::createTree(const vector<Point>& data, unsigned int max_leaf_size)
{
    this->data.assign(data.begin(), data.end());
    this->maxLeafSize = max_leaf_size;
    this->maxLevel= ceil(log2(data.size()));
    
    vector<int> indices;
    for(int i = 0; i < this->data.size(); i++)
    {
        indices.push_back(i);
    }

    this->root = new KDNode();
    this->root->box = this->buildBoundingBox(indices);
    this->buildTree(indices, this->root, 0);

    return true;
}

//// helper function for creating the tree using point indices
bool KDTree::buildTree(const vector<int> & indices, KDNode* & node, unsigned int level)
{
    assert(node != NULL);

    if(level >= maxLevel || indices.size() <= maxLeafSize)
    {
        node->isLeaf = true;
        node->level = level;
        node->dataIndex = indices;
        node->box = this->buildBoundingBox(indices);
        return true;
    }

    node->level = level;
    node->dataIndex = indices;

    // randomly select split dimensions
    KDTreeSplitAxis splitDim = (KDTreeSplitAxis)(rand() % Point::dimension);
    node->splitDim = splitDim;
    
    vector<double> one_dim_values;
    for(int i = 0; i < indices.size(); i++)
    {
        Point &p = data[indices[i]];
        double val = splitDim == KDTreeSplitAxisX ? p.x : p.y;
        one_dim_values.push_back(val);
    }

    // median value as split value, every value before the meidan is less than the median, every value after the median is larger than the median;
    size_t n = one_dim_values.size() / 2;
    std::nth_element(one_dim_values.begin(), one_dim_values.begin() + n, one_dim_values.end());

    node->splitValue = one_dim_values[n];
    node->box = this->buildBoundingBox(indices);

    vector<int> leftIndices;
    vector<int> rightIndices;
    for(int i = 0; i < indices.size(); i++)
    {
        Point p = data[indices[i]];
        double v = splitDim == KDTreeSplitAxisX ? p.x : p.y;
        if(v < node->splitValue)
        {
            leftIndices.push_back(i);
        }
        else
        {
            rightIndices.push_back(i);
        }
    }

    if(leftIndices.size() != 0)
    {
        KDNode* left = new KDNode();
        this->buildTree(leftIndices, left, level + 1);
        node->left = left;
    }

    if(rightIndices.size() != 0)
    {
        KDNode *right = new KDNode();
        this->buildTree(rightIndices, right, level+1);
        node->right = right;
    }

    return true;
}

Box KDTree::buildBoundingBox(const vector<int> & indices) const
{
    Point minP = data[indices.front()];
    Point maxP = minP;

    for(int i = 1; i < indices.size(); ++i)
    {
        Point p = data[indices[i]];
        
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

    return Box(minP.x, minP.y, maxP.x, maxP.y);

}

const KDNode* KDTree::getLeafNode(const Point &query_point, const KDNode *node)const
{
    if(node->isLeaf) return node;

    KDTreeSplitAxis dim = node->splitDim;
    double split_value = node->splitValue;
    double value = dim == KDTreeSplitAxisX ? query_point.x : query_point.y;

    if(value < split_value && node->left)
    {
        return this->getLeafNode(query_point, node->left);
    }
    else if(node->right)
    {
        return this->getLeafNode(query_point, node->right);
    }
    else
    {
        return NULL;
    }
}

bool KDTree::kNNQuery(const Point &query, const int K, vector<int> &indices, vector<double>&disSq) const
{
    double maxDisSQ = numeric_limits<double>::max();
    priority_queue<DistanceIndex> distanceQueue;

    stack<KDNode*> candidates;
    KDNode* cur = root;

    // travel to the leaf node
    while(cur != NULL)
    {
        candidates.push(cur);
        if(cur ->isLeaf) break;
        
        KDTreeSplitAxis dim = cur->splitDim;
        double split_value = cur->splitValue;
        double value = dim == KDTreeSplitAxisX ? query.x : query.y;

        if(value < split_value && cur->left != NULL)
        {
            cur = cur->left;
        }
        else if(cur->right != NULL)
        {
            cur = cur->right;
        }
    }

    set<KDNode*> visited;
    this->query(query, K, candidates, visited, distanceQueue, maxDisSQ);

    indices.resize(K);
    disSq.resize(K);

    int num = K -1;
    while(!distanceQueue.empty())
    {
        DistanceIndex top = distanceQueue.top();
        distanceQueue.pop();
        indices[num] = top.index;
        disSq[num] = top.distance;
        num--;
    }

    return true;
}

bool KDTree::query(const Point &query, const int K, stack<KDNode *> &nodes, set<KDNode *> &visited, priority_queue<DistanceIndex> &priorityQueue, double &maxDis)const
{
    while(!nodes.empty())
    {
        KDNode* cur = nodes.top();
        nodes.pop();

        //already visited node
        if(visited.find(cur)!= visited.end()) continue;

        visited.insert(cur);
        if(cur->isLeaf)
        {
            this->exploreLeafNode(query, K, priorityQueue, maxDis, cur);
        }
        else
        {
            // internal node
            if(cur->left)
            {
                double disSQ = cur->left->box.distanceSQ(query);
                if(disSQ < maxDis)
                {
                    nodes.push(cur->left);
                }
            }
            if(cur->right)
            {
                double disSQ = cur->left->box.distanceSQ(query);
                if(disSQ < maxDis)
                {
                    nodes.push(cur->right);
                }
            }
        }
    }
    return true;
}

bool KDTree::exploreLeafNode(const Point &query, const int K, priority_queue<DistanceIndex> &priorityQueue, double &maxDis, const KDNode *node)const
{
    for(int i = 0; i < node->dataIndex.size(); i++)
    {
        int index = node->dataIndex[i];
        Point point = data[index];
        double dist_sq = point.squareDistance(query);
        if(priorityQueue.size() < K || dist_sq < maxDis)
        {
            DistanceIndex di(index, dist_sq);
            priorityQueue.push(di);
            if(priorityQueue.size() > K)
            {
                priorityQueue.pop();
            }
            maxDis = priorityQueue.top().distance;
        }
    }
    return true;
}


#endif /* KD_TREE_H */
