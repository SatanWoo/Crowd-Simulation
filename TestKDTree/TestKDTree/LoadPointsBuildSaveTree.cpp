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

  vector<vector<double> > dataset;
  ReadData rd1("sample_data.txt");
  dataset=rd1.allDataPointsVec;


  /** Build a KD Tree **/
  KD_tree kd;
  int max_leaf_size =1;
  kd.create_tree(dataset, max_leaf_size);

  /** Save the KD Tree to disk **/
  //string KDTree_Storage_fileName="KD_tree_storage.txt";
  kd.save_tree_to_file("KD_tree_storage.txt");

  cout<<"The required application 1 has been successful, thank you! Please check the KD_tree_storage.txt for the KD tree "<<endl;

  return 0;

}
