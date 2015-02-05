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
 * This is just an header file for the gradientMex.cpp created by Piotr Dollar.
 * Take attention when mergin with other versions of the Piotr's toolbox, as 
 * stated in the README.
 */
#ifndef GRADIENTMEX_HPP_
#define GRADIENTMEX_HPP_

#include <cmath>
#include "wrappers.hpp"
#include <cstring>
#include "sse.hpp"

#define PI 3.1415926535897931f

// compute x and y gradients for just one column (uses sse)
void grad1( float *I, float *Gx, float *Gy, int h, int w, int x );

// compute x and y gradients at each location (uses sse)
void grad2( float *I, float *Gx, float *Gy, int h, int w, int d );

// build lookup table a[] s.t. a[dx/2.02*n]~=acos(dx)
float* acosTable();

// compute gradient magnitude and orientation at each location (uses sse)
void gradMag( float *I, float *M, float *O, int h, int w, int d );

// normalize gradient magnitude at each location (uses sse)
void gradMagNorm( float *M, float *S, int h, int w, float norm );

// helper for gradHist, quantize O and M into O0, O1 and M0, M1 (uses sse)
void gradQuantize( float *O, float *M, int *O0, int *O1, float *M0, float *M1,
  int nOrients, int nb, int n, float norm );

// compute nOrients gradient histograms per bin x bin block of pixels
void gradHist( float *M, float *O, float *H, int h, int w,
  int bin, int nOrients, bool softBin );

// compute HOG features given gradient histograms
void hog( float *H, float *G, int h, int w, int bin, int nOrients, float clip );

#endif /* GRADIENTMEX_HPP_ */
