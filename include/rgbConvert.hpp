/*******************************************************************************
* Pedestrian Detector v0.1    2013-08
*
* Fabio Reis
* [freis-at-isr.ist.utl.pt]
*
* Please email me if you find bugs, or have suggestions or questions!
* Licensed under the Simplified BSD License [see external/bsd.txt]
*******************************************************************************/

#ifndef RGBCONVERT_HPP_
#define RGBCONVERT_HPP_

#include "rgbConvertMex.hpp"

/*
 * Wrapper for the Piotr Dollar rgbConvert function
 */
float* rgbConvertMeta(float *image, int height, int width, int channels,
		int colorSpace, int norm);

#endif /* RGBCONVERT_HPP_ */
