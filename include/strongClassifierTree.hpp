/*******************************************************************************
* Pedestrian Detector v0.1    2013-08
*
* Fabio Reis and Matteo Taiana
* [freis-at-isr.ist.utl.pt] and [mtaiana-at-isr.ist.utl.pt]
* 
* Original code by Matteo Taiana targeted for Matlab.
*
* Please email us if you find bugs, or have suggestions or questions!
* Licensed under the Simplified BSD License [see external/bsd.txt]
*******************************************************************************/
#ifndef STRONGCLASSIFIERTREE_HPP_
#define STRONGCLASSIFIERTREE_HPP_

/*
 * System includes
 */
#include <cstdio>
#include <cstdlib>
#include <iostream>
#include <vector>
#include <list>

/*
 * Our includes
 */
#include "chnsCompute.hpp"
#include "chnsPyramid.hpp"
#include "readFiles.hpp"

class classifierInput {
public:
	ClassData *classData;
	ClassRectangles *classRect;

	float widthOverHeight;	              //[0.41]
	int shrinkFactor;		                  //[4]

	int theoreticalWindowWidth; 		      //[52]
	int theoreticalWindowHeight;		      //[128]
	float theoreticalActiveWindowWidth;   //[39.36]
	float theoreticalActiveWindowHeight;  //[96]

	int windowWidth;
	int windowHeight;
	int activeWindowWidth;
	int activeWindowHeight;

	int windowHorizontalPadding;
	int windowVerticalPadding;
  int theoreticalHorizontalPadding;     //[7] before shrink
  int theoreticalVerticalPadding;       //[16] before shrink
	int imageHorizontalPadding;
	int imageVerticalPadding;
	int verticalSuperPadding;			        //[0]
	int horizontalSuperPadding;			      //[0]


	int nWeakClassifiers;                 //[1000]
	int nClassifiers;					            //[1000] this has no effect on the
										                    //learner, only on the detector
										        
	int nFeatures; 						            //[3000] GAS has x2
	int nRectRows; 						            //[3000] GAS has x2
	int nRectCols;                        //[5]
	int nBaseFeatures; 					          //[3000] //WARNING hardcoding
	int nExtraFeatures;					          //[10] //WARNING hardcoding

	bool returnFeatures;				          //[true]
	bool returnVotes;					            //[false]
	bool verbose;						              //[false]
	bool performNMS;					            //[true]


  classifierInput(ClassData *classifier,
                  ClassRectangles *rect,
                  bool _verbose,
                  float _widthOverHeight,
                  int _shrinkFactor,
                  int _theoreticalWindowWidth,
                  int _theoreticalWindowHeight,
                  float _theoreticalActiveWindowWidth,
                  float _theoreticalActiveWindowHeight,
                  int _nBaseFeatures,
                  int _nExtraFeatures
                 );
};

/*
 * Functions to access classifier data
 */
double alpha(int row);
int feature(int row);
double threshold(int row);
int direction(int row);
int featureSat(int row);
double thresholdSat(int row);
int directionSat(int row);
int featureNotSat(int row);
double thresholdNotSat(int row);
int directionNotSat(int row);

/*
 * Actual Strong Classifier Tree Algorithm
 */
const double magicThreshold = -3.0;
vector<cv::Rect_<int> >* sctRun(pyrOutput *outputPyr, classifierInput *cInput);

using namespace std;

#endif /* STRONGCLASSIFIERTREE_HPP_ */
