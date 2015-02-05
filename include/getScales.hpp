/*******************************************************************************
* Pedestrian Detector v0.1    2013-08
*
* Fabio Reis
* [freis-at-isr.ist.utl.pt]
*
* Please email me if you find bugs, or have suggestions or questions!
* Licensed under the Simplified BSD License [see external/bsd.txt]
*******************************************************************************/
#ifndef GETSCALES_HPP_
#define GETSCALES_HPP_

/*
 * System includes
 */
#include <cmath>
#include <cstdlib>

int getScales(float *&scales, float *&scaleshw, int nPerOct, int nOctUp,
		int* minDs, int shrink, int* sz);

#endif /* GETSCALES_HPP_ */
