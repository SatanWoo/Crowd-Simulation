/**
 * File: PrintAllDataID.cpp
 * Author: Lili Meng (lilimeng1103@gmail.com)
 * Print out the ID of the point where the ID is the row in the original file(0-based)
 * The format is very simple:
 * A00 A01 A02 ... A0N
 * A10 A11 A12 ... A1N
 */

#include "ReadData1.h"


int main() {

    string fileName;
    fileName = "sample_data.txt";
    ReadData l(fileName);

    
    cout<<"The Data ID from "<<fileName<<" "<<"is the following:"<<endl;
    l.printDataID();


    return 0;
}
