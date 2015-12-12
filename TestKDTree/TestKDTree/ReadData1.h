/**
 * File: ReadData.h
 * Author: Lili Meng (lilimeng1103@gmail.com)
 * read the line and column data from .txt file
 */

#ifndef READDATA_H
#define READDATA_H

#include <iostream>
#include <vector>
#include <fstream>
#include <iomanip>
#include <sstream>

using namespace std;

class ReadData {
public:
    ReadData(string filename);
    vector<vector<double>> allPoints;
    size_t numOfElements();
    size_t dimension();
    /**
     * Print out the ID of point where the ID is the row in the original file (0-based)
     */
    //void printDataID();
};

ReadData::ReadData( string filename )
{
    std::ifstream fin(filename.c_str(),std::ios::in);
    if (!fin.is_open())
    {
        cout << "cannot open the file" << endl;
        return;
    }

    istringstream istr;
    string str;
    double one, two;
    while(getline(fin,str))
    {
        vector<double>data;
        istr.str(str);
        istr >> one >> two;
        
        data.push_back(one);
        data.push_back(two);
        
        allPoints.push_back(data);
        istr.clear();
        str.clear();
    }
    fin.close();
}

size_t ReadData::numOfElements()
{
    return allPoints.size();
}

size_t ReadData::dimension()
{
    return 2;
}

//void ReadData::printDataID()
//{
//    size_t numOfElements = allPoints.size();
//    for(int i = 0; i < numOfElements; i++)
//    {
//        Point p = allPoints[i];
//        cout << "PointID is " << "A" << i << "\t" << setprecision(20) << "Value is   " << p.x << ":" << p.y << endl;
//    }
//}
#endif // READDATA_H
