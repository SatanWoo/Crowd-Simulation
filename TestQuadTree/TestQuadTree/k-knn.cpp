//
//  main.cpp
//  TestQuadTree
//
//  Created by z on 15/12/13.
//  Copyright © 2015年 SatanWoo. All rights reserved.
//

#include <iostream>
#include "QuadTree.h"
#include "ReadData1.h"

using namespace std;

int main(int argc, const char * argv[])
{
    ReadData rd1("/Users/z/Documents/Crowd\ Simulation/Crowd\ Simulation/TestKDTree/TestKDTree/sample_data.txt");
    
    //query_point
    vector<double> query_point;
    vector<Point> query_point_dataset;
    ReadData rd2("/Users/z/Documents/Crowd\ Simulation/Crowd\ Simulation/TestKDTree/TestKDTree/query_points.txt");
    query_point_dataset = rd2.allPoints;
    
    QuadTree tree(rd1.allPoints, 4);
    
    cout << "Data Set is " << rd1.allPoints.size() << std::endl;
    
    vector<int> indices1;
    vector<double> squared_distances1;
    
    ofstream fout("/Users/z/Documents/Crowd\ Simulation/Crowd\ Simulation/Quad-K-Knn.txt");
    
    /** Exact kNN Search **/
    cout<<"-------------------------------------------------------------------------------"<<endl;
    cout<<"*********Using Exact k Nearest Neighbor Search, The following are Results******"<<endl;
    
    for (int K = 1; K <= rd1.allPoints.size(); K++) {
        clock_t start = clock();
        for(int i = 0; i < query_point_dataset.size(); i++)
        {
            tree.KNNSearch(query_point_dataset[i], K, indices1, squared_distances1);
//            for (int j = 0; j < K; j++)
//            {
//                cout << "For the number row  " << i << "  query point, Using Exact kNN Search 3 Nearest Neigbour : The number "<< j + 1 << " nearest neighbor index is  "<< indices1[j] << endl;
//            }
        }
        clock_t end = clock();
        double time = 1000.0f * (end - start) / CLOCKS_PER_SEC;
        fout << "K: " << K << "  Time: " << time << std::endl;
    }
    
    fout.close();
    
    cout << "The required Quad-K-KNN relation test has been executed successful, thank you!" << endl;
    
    return 0;
}
