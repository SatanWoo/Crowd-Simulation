/**
 * File:  kNNSearch.cpp
 * Author Lili Meng (lilimeng1103@gmail.com)
 * The required application 2: Read a simple text fle and reports the top 3 nearest neighbors using both exact search and BBF approximate search
 */

#include <iostream>
#include "KD_tree.h"
#include "ReadData1.h"

using namespace std;


int main(int argc, const char * argv[])
{
    int K = 3;
    ReadData rd1("/Users/z/Documents/Crowd\ Simulation/Crowd\ Simulation/TestKDTree/TestKDTree/sample_data.txt");

    //query_point
    vector<double> query_point;
    vector<Point> query_point_dataset;
    ReadData rd2("/Users/z/Documents/Crowd\ Simulation/Crowd\ Simulation/TestKDTree/TestKDTree/query_points.txt");
    query_point_dataset = rd2.allPoints;

    KDTree tree(rd1.allPoints, 4);

    vector<int> indices1;
    vector<double> squared_distances1;

    /** Exact kNN Search **/
    cout<<"-------------------------------------------------------------------------------"<<endl;
    cout<<"*********Using Exact k Nearest Neighbor Search, The following are Results******"<<endl;
    for(int i=0; i < query_point_dataset.size(); i++)
    {
        tree.kNNQuery(query_point_dataset[i], K, indices1, squared_distances1);
        for (int j = 0; j < K; j++)
        {
            cout << "For the number row  " << i << "  query point, Using Exact kNN Search 3 Nearest Neigbour : The number "<< j + 1 << " nearest neighbor index is  "<< indices1[j] << endl;
        }
    }

//    vector<int> indices2;
//    vector<double> squared_distances2;
//
//    /** BBF Approximate kNN Search **/
//    cout<<"--------------------------------------------------------------------------------"<<endl;
//    cout<<"--------------------------------------------------------------------------------"<<endl;
//    cout<<"--------------------------------------------------------------------------------"<<endl;
//    cout<<"***Using BBF Approximate k Nearest Neighbors Search, the following are Results**"<<endl;
//
//    for(int i=0; i<query_point_dataset.size(); i++)
//    {
//        /** If the max_epoch is equal or larger than the size of all leaves, the result is the same with the exact kNN search**/
//        int max_searched_leaf_number=1000;
//        tree.bbf_kNN_query(query_point_dataset[i], K, indices2, squared_distances2, max_searched_leaf_number);
//
//        for (int j = 0; j<K; j++)
//        {
//            cout<<"For the number row  "<<i<<"  query point, Using BBF approximate kNN Search 3 Nearest Neigbour : The number "<<j+1<<" nearest neighbor index is  "<<indices2[j]<<endl;
//        }
//
//    }
//
//    cout<<"The required application 2 has been successful, thank you!"<<endl;

    return 0;
}
