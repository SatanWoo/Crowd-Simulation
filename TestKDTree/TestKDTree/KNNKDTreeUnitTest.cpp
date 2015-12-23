/**
 * File: KNNKDTreeUnitTest.cpp
 * Author Lili Meng (lilimeng1103@gmail.com) based on the test-harness.cpp code by Keith Schwarz (htiek@cs.stanford.edu)
 * Test CreateKDTree, ExactKNearestNeighbor and ApproximateKNearestNeighbor
 */

#include <iostream>
#include "KD_tree.h"
#include "ReadData1.h"

using namespace std;

/* These flags control which tests will be run.*/


#define CreateKDTreeTestEnabled          1 // Step one checks

#define ExactKNearestNeighborTestEnabled      1 // Step two checks

#define ApproximateKNearestNeighborTestEnabled      1 // Step three checks

/* Utility function that pauses until the user hits ENTER. */
void PressEnterToContinue() {
    /* Use getline to stall until receiving input. */
    string line;
    getline(cin, line);
}

/* This function is what the test suite uses to ensure that the KDTree works
 * correctly. It takes as parameters an expression and description, along
 * with a file and line number, then checks whether the condition is true.
 * If so, it prints that the test passed. Otherwise, it reports that the test
 * fails and points the caller to the proper file and line.
 */

void DoCheckCondition(bool expr, const string& rationale, const string& file, int line) {
    /* It worked! Congrats. */
    if (expr) {
    cout << "PASS: "<< rationale << endl;
    return;
    }

    /* Uh oh! The test failed! */
    cout << "FAIL: " << rationale << endl;
    cout <<"   Error at " << file <<", line" << line << endl;
    cout <<"   (ENTER to continue)" << endl;

    /* Pause so that the test fail stands out. */
    PressEnterToContinue();
}

/* Reports that an unexpected error occurred that caused a test to fail. */
void FailTest(const exception& e) {
    cerr << "Please be more patient, TEST FAILED: Unexpected exception: " <<e.what() << endl;
    PressEnterToContinue();
}

 /* This macro takes in an expression and a string, then invokes
  * DoCheckCondition passing in the arguments along with the file
  * and line number on which the macro was called. This makes it
  * easier to track down the source of bugs if a test case should
  * fail.
  */
#define CheckCondition(expr, rationale) DoCheckCondition(expr, rationale, __FILE__, __LINE__)

/* Utility function to delimit the start and end of test cases. */
void PrintBanner(const string& header) {
    cout << "\nBeginning test: " << header << endl;
    cout << setw(40) << setfill('-') << "" << setfill(' ') <<endl;
}


/* Utility function to signal that a test isn't begin run. */
void TestDisabled(const string& header) {
  cout << "== Test " << header << " NOT RUN: press ENTER to continue ==" << endl;

  /* Pause for the user to hit enter. */
  PressEnterToContinue();
}

/* Utility function to signal the end of a test. */
void EndTest() {
  cout << "== end of test: Thank you! press ENTER to continue ==" << endl;
  PressEnterToContinue();
}


//A brute force method to check the distances
double distance_sq(const vector<double> & data0, const vector<double> & data1)
{
    assert(data0.size() == data1.size());
    double sq = 0.0;
    for (int i=0; i<data0.size(); i++)
    {
        double dif = data0[i]-data1[i];
        sq +=dif*dif;
    }
    return sq;
}

/* Basic test: Can we build the tree and look up the elements it contains? */
void CreateKDTreeTest() try {
#if CreateKDTreeTestEnabled
  PrintBanner("KDTree Test");

  /* Construct the KDTree. */
   /* Read Data */
  vector<vector<double> > dataset;
  ReadData rd1("sample_data.txt");
  int N = rd1.get_num_of_elements();
  int dim = rd1.get_num_of_dimensions();
  dataset=rd1.allDataPointsVec;

  KD_tree kd;
  CheckCondition(kd.empty(),   "New KD tree is empty.");
   /* Add some elements. */
  bool completedkd=kd.create_tree(dataset, 4);

  CheckCondition(completedkd==true, "KDTree construction completed.");

  /* Check basic properties of the KDTree. */
  CheckCondition(kd.dimension() == 128, "Dimension is 128.");

  /* Check basic properties again. */
  CheckCondition(kd.size() == 1000, "After adding 1000 elements, KDTree has size 1000.");
  CheckCondition(!kd.empty(),    "After adding  elements, KDTree is not empty.");


  EndTest();
#else
  TestDisabled("KDTreeTest");
#endif
} catch (const exception& e) {
  FailTest(e);
}

void ExactKNearestNeighborTest() try {
#if ExactKNearestNeighborTestEnabled
  PrintBanner("Exact k Nearest Neighbor Test");

    int K = 3;

    vector<vector<double> > dataset;
    ReadData rd1("sample_data.txt");
    int N = rd1.get_num_of_elements();
    int dim = rd1.get_num_of_dimensions();
    dataset=rd1.allDataPointsVec;

    //query_point
    vector<double> query_point;
    vector<vector<double> > query_point_dataset;
    ReadData rd2("query_points.txt");
    int N2 = rd2.get_num_of_elements();
    int dim2 = rd2.get_num_of_dimensions();
    query_point_dataset=rd2.allDataPointsVec;
    query_point=query_point_dataset[1];

    KD_tree_node* root;
    KD_tree tree;
    tree.create_tree(dataset, 4);


    vector<int> indices;
    vector<double> squared_distances;
    int max_epoch = 1000;
    tree.bbf_kNN_query(query_point, K, indices, squared_distances, max_epoch);


    // brute force
    unordered_map<double, int> brute_force_htable;
    vector<double> brute_force_vec;

    for (int i = 0; i< N; i++)
    {
        double dist = distance_sq(query_point, dataset[i]);

        brute_force_htable.insert({dist, i});
        brute_force_vec.push_back(dist);
    }

    std::sort(brute_force_vec.begin(), brute_force_vec.end());

    /**Compare the BBF Search with the Brute-Force Method */
    for (int i = 0; i< K; i++)
    {
        CheckCondition( indices[i]==brute_force_htable[brute_force_vec[i]], "Comparing with the Brute-force method, the Exact K-Nearest Neighbour search by KD-Tree program is correct");
    }

  EndTest();
#else
  TestDisabled("ExactKNearestNeighborTest");
#endif
} catch (const exception& e) {
  FailTest(e);
}


void ApproximateKNearestNeighborTest() try {
#if ApproximateKNearestNeighborTestEnabled
  PrintBanner("Approximate k Nearest Neighbor Test");

    int K = 3;

    vector<vector<double> > dataset;
    ReadData rd1("sample_data.txt");
    int N = rd1.get_num_of_elements();
    int dim = rd1.get_num_of_dimensions();
    dataset=rd1.allDataPointsVec;

    //query_point
    vector<double> query_point;
    vector<vector<double> > query_point_dataset;
    ReadData rd2("query_points.txt");
    int N2 = rd2.get_num_of_elements();
    int dim2 = rd2.get_num_of_dimensions();
    query_point_dataset=rd2.allDataPointsVec;
    query_point=query_point_dataset[1];

    KD_tree_node* root;
    KD_tree tree;
    tree.create_tree(dataset, 4);


    vector<int> indices;
    vector<double> squared_distances;
    tree.kNN_query(query_point, K, indices, squared_distances);


    // brute force
    unordered_map<double, int> brute_force_htable;
    vector<double> brute_force_vec;

    for (int i = 0; i< N; i++)
    {
        double dist = distance_sq(query_point, dataset[i]);

        brute_force_htable.insert({dist, i});
        brute_force_vec.push_back(dist);
    }

    std::sort(brute_force_vec.begin(), brute_force_vec.end());

    /**Compare the BBF Search with the Brute-Force Method, In testing, the max_epoch was set 1000 so that it's almost the exact search */
    for (int i = 0; i< K; i++)
    {
        CheckCondition( indices[i]==brute_force_htable[brute_force_vec[i]], "Comparing with the Brute-force method, the Approximate K-Nearest Neighbour search by KD-Tree program is correct");
    }

  EndTest();
#else
  TestDisabled("ExactKNearestNeighborTest");
#endif
} catch (const exception& e) {
  FailTest(e);
}


int main() {
  /* Step One Tests */
  CreateKDTreeTest();

  /* Step Two Tests */
  ExactKNearestNeighborTest();

  /* Step Three Tests */
  ApproximateKNearestNeighborTest();



#if (CreateKDTreeTestEnabled && \
     ExactKNearestNeighborTestEnabled &&\
     ApproximateKNearestNeighborTestEnabled)

  cout << "All tests completed! Thank you for your patience!" << endl << endl;
#else
  cout << "Not all tests were run.  Enable the rest of the tests, then run again." << endl << endl;
#endif

  PressEnterToContinue();
}
