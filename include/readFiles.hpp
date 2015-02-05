/*******************************************************************************
* Pedestrian Detector v0.1    2013-08
*
* Fabio Reis
* [freis-at-isr.ist.utl.pt]
*
* Please email me if you find bugs, or have suggestions or questions!
* Licensed under the Simplified BSD License [see external/bsd.txt]
*******************************************************************************/

#ifndef READFILES_HPP_
#define READFILES_HPP_

/*
 * System includes
 */
#include <cstdio>
#include <cstdlib>
#include <iostream>
#include <fstream>
#include <string>
#include <vector>

using namespace std;

/*
 * Helper functions that reads the classifier data
 */

/*
 * Classifier rectangles
 */

class ClassRectangles {
public:
  ClassRectangles(string _fname, int _nRows, int _nCols);
  ~ClassRectangles();
  
  int* rectangles;
  int nRows; // Number of Features
  int nCols; // Number of properties per rectangle
  
private:
  void readRectangles(string fname);
};

/*
 * Classifier data
 */

class ClassData {
public:
  ClassData(string _fname, int _nRows, int _nCols);
  ~ClassData();
  
  double* classifiers;
  int nRows; // Number of weak classifiers
  int nCols; // Number of columns per classifier
  
private:
  void readClassifierData(string fname);
};



#endif /* READFILES_HPP_ */
