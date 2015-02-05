/*******************************************************************************
* Pedestrian Detector v0.1    2013-08
*
* Fabio Reis
* [freis-at-isr.ist.utl.pt]
*
* Please email me if you find bugs, or have suggestions or questions!
* Licensed under the Simplified BSD License [see external/bsd.txt]
*******************************************************************************/

#ifndef OPENCVINTERFACE_HPP_
#define OPENCVINTERFACE_HPP_

/*
 * System includes
 */
#include <cstdio>
#include <cstdlib>
#include <iostream>
#include <fstream>

/*
 * Piotr Dollar's include functions
 */
#include "sse.hpp"
#include "wrappers.hpp"

/*
 * OpenCV Includes
 */
#include <opencv2/opencv.hpp>

using namespace cv;

/*
 * Some helper functions for reading image files to be used in the Piotr Dollar
 * toolbox, to print to the console, write to a MatLab compatibile file, etc.
 */
Mat* convertToMat(const float* data, int height, int width, int channels,
		int misalign);

Mat* convertToMatChannel(const float* data, int height, int width, int channels,
		int channel, int misalign);

float* convertFromMat(const Mat& data, int height, int width, int channels,
		int misalign);

void writeToMatlab(const float* data, int height, int width, int channels,
		int misalign, string filename, string name);

void writeToConsole(const float* data, int height, int width, int channels,
		int misalign);

#endif /* OPENCVINTERFACE_HPP_ */
