/*******************************************************************************
* Pedestrian Detector v0.1    2013-08
*
* Fabio Reis
* [freis-at-isr.ist.utl.pt]
*
* Please email me if you find bugs, or have suggestions or questions!
* Licensed under the Simplified BSD License [see external/bsd.txt]
*******************************************************************************/

#include "../include/readFiles.hpp"


/*
 * Classifier rectangles
 */
 
ClassRectangles::ClassRectangles(string _fname, int _nRows, int _nCols){
  nRows = _nRows;
  nCols = _nCols;
  readRectangles(_fname);
};
  
ClassRectangles::~ClassRectangles(){
 delete(rectangles);
};
  
void ClassRectangles::readRectangles(string _fname){
    ifstream file (_fname.c_str());
    string valueS;
    int i = 0, j = 0, value;
    rectangles = new int[nRows*nCols];

    while( file.good() && i < nRows){
      if (j < nCols - 1)
          getline ( file, valueS, ' ' ); // read a string until next ' '
      else
          getline ( file, valueS );  // read a string until next '\n'

      value = atoi(valueS.c_str()); // converts it to int

      rectangles[i*nCols + j] = value;

      j++;

      if (j == nCols){
          i++;
          j = 0;
      }
    }

    file.close();
}


/*
 * Classifier data
 */
 
ClassData::ClassData(string _fname, int _nRows, int _nCols){
  nRows = _nRows;
  nCols = _nCols;
  readClassifierData(_fname);
};
  
ClassData::~ClassData(){
 delete(classifiers);
};

void ClassData::readClassifierData(string _fname){
    ifstream file (_fname.c_str());
    string valueS;
    int i = 0, j = 0;
    double value;
    classifiers = new double[nRows*nCols];

    if (file.good())
	    getline(file, valueS); // Take out the first line

    while( file.good() && i < nRows){
	    if (j < nCols - 1)
	        getline ( file, valueS, ' ' ); // read a string until next ' '
	    else
	        getline ( file, valueS );  // read a string until next '\n'

    	value = atof(valueS.c_str()); // converts it to int

    	classifiers[i*nCols + j] = value;

    	j++;

	    if (j == nCols){
		    i++;
		    j = 0;
	    }
    }

    file.close();
}
