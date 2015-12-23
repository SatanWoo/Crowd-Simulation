/**
 * File:  LoadPointsBuildSaveTree.cpp
 * Author Lili Meng (lilimeng1103@gmail.com)
 * The required application 1: Load points from a simple text file, build a KD Tree and save the KD Tree to disk
 */

#include <iostream>
#include "KD_tree.h"
#include "ReadData1.h"

int main()
{
    /**Load points from a simple text file**/
    vector<Point> dataset;
    ReadData rd1("/Users/z/Documents/Crowd\ Simulation/Crowd\ Simulation/TestKDTree/TestKDTree/sample_data.txt");
    dataset = rd1.allPoints;
    
    //query_point
    vector<double> query_point;
    vector<Point> query_point_dataset;
    ReadData rd2("/Users/z/Documents/Crowd\ Simulation/Crowd\ Simulation/TestKDTree/TestKDTree/query_points.txt");
    query_point_dataset = rd2.allPoints;
    
    cout << "size " << rd1.allPoints.size() << endl;
    
    vector<int> indices1;
    vector<double> squared_distances1;
    int K = 38;
    ofstream fout("/Users/z/Documents/Crowd\ Simulation/Crowd\ Simulation/Leaf-KD-KNN.txt");
    
    for (int leafSize = 1; leafSize <= 50; leafSize++) {
         /** Build a KD Tree **/
        KDTree tree(rd1.allPoints, 1);
        clock_t start = clock();
        for(int i = 0; i < query_point_dataset.size(); i++)
        {
            tree.KNNQuery(query_point_dataset[i], K, indices1, squared_distances1);
        }
        clock_t end = clock();
        double time = 1000.0f * (end - start) / CLOCKS_PER_SEC;
        fout << "Leaf: " << leafSize << "  Time: " << time << endl;
    }
    
    fout.close();

    cout << "The requried leaf-knn relation test has been exectued successfully" << endl;

    return 0;
}
