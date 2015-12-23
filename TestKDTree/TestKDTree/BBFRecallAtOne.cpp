/**
 * File:  BBFRecallAtOne.cpp
 * Author Lili Meng (lilimeng1103@gmail.com)
 * The Recall of BBF Approximate Search at 1 nearest neighbor for 10 query points from a dataset of 1000 points
 */

#include <iostream>
#include "KD_tree.h"
#include "ReadData1.h"

using namespace std;


int main(int argc, const char * argv[])
{
    int K = 1;

    vector<vector<double> > dataset;
    ReadData rd1("sample_data.txt");
    dataset=rd1.allDataPointsVec;

    //query_point
    vector<double> query_point;
    vector<vector<double> > query_point_dataset;
    ReadData rd2("query_points.txt");
    query_point_dataset=rd2.allDataPointsVec;

    KD_tree_node* root;
    KD_tree tree;
    int max_leaf_size =2;
    tree.create_tree(dataset,max_leaf_size);

    vector<int> indices1;
    vector<double> squared_distances1;
    vector<int> trueSearch(10);
    for(int i=0; i<query_point_dataset.size(); i++)
    {
        tree.kNN_query(query_point_dataset[i], K, indices1, squared_distances1);
        //cout<<indices1[0]<<endl;
        trueSearch[i]=indices1[0];
        cout<<trueSearch[i]<<endl;

    }
    ofstream fout("RecallAtOne.csv");
    vector<int> indices2;
    vector<double> squared_distances2;

    double correct_number[1001][10];
    double RecallAtOne[1001];
    double sum[1001];


   for(int max_searched_leaf_number=1; max_searched_leaf_number<=1000; max_searched_leaf_number++)
    /** If the max_epoch is equal or larger than the size of all leaves, the result is the same with the exact kNN search**/
    {
        for(int j=0; j<query_point_dataset.size(); j++)
        {
            tree.bbf_kNN_query(query_point_dataset[j], K, indices2, squared_distances2, max_searched_leaf_number);
            if(indices2[0]==trueSearch[j])
            {
                correct_number[max_searched_leaf_number][j]++;
            }
            sum[max_searched_leaf_number]+=correct_number[max_searched_leaf_number][j];
        }

        RecallAtOne[max_searched_leaf_number]=sum[max_searched_leaf_number]/10.000;
        cout<<max_searched_leaf_number<<"\t"<<setprecision(5)<<RecallAtOne[max_searched_leaf_number]<<endl;
        fout<<max_searched_leaf_number<<"\t"<<setprecision(5)<<RecallAtOne[max_searched_leaf_number]<<endl;
   // cout<<"The Recall of BBF Approximate Search at 1 nearest neighbor for 10 query points from a dataset of 1000 points is: "<<setprecision(3)<<RecallAtOne<<endl;
    }

 return 0;
}
