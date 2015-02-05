/*******************************************************************************
* Pedestrian Detector v0.1    2013-08
*
* Fabio Reis
* [freis-at-isr.ist.utl.pt]
*
* Please email me if you find bugs, or have suggestions or questions!
* Licensed under the Simplified BSD License [see external/bsd.txt]
*******************************************************************************/

/*
 * This is just an header file for the convConst.cpp created by Piotr Dollar.
 * Take attention when mergin with other versions of the Piotr's toolbox, as 
 * stated in the README.
 */
#ifndef CONVCONST_HPP_
#define CONVCONST_HPP_

/*
 * System includes
 */
#include <cstring>

/*
 * Piotr Dollar includes
 */
#include "wrappers.hpp"
#include "sse.hpp"

// convolve two columns of I by ones filter
void convBoxY( float *I, float *O, int h, int r, int s );

// convolve I by a 2r+1 x 2r+1 ones filter (uses SSE)
void convBox( float *I, float *O, int h, int w, int d, int r, int s );

// convolve single column of I by [1; 1] filter (uses SSE)
void conv11Y( float *I, float *O, int h, int side, int s );

// convolve I by [1 1; 1 1] filter (uses SSE)
void conv11( float *I, float *O, int h, int w, int d, int side, int s );

// convolve one column of I by a 2rx1 triangle filter
void convTriY( float *I, float *O, int h, int r, int s );

// convolve I by a 2rx1 triangle filter (uses SSE)
void convTri( float *I, float *O, int h, int w, int d, int r, int s );

// convolve one column of I by [1 p 1] filter (uses SSE)
void convTri1Y( float *I, float *O, int h, float p, int s );

// convolve I by [1 p 1] filter (uses SSE)
void convTri1( float *I, float *O, int h, int w, int d, float p, int s );

// convolve one column of I by a 2rx1 max filter
void convMaxY( float *I, float *O, float *T, int h, int r );

// convolve every column of I by a 2rx1 max filter
void convMax( float *I, float *O, int h, int w, int d, int r );

#endif /* CONVCONST_HPP_ */
