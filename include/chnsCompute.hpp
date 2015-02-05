/*******************************************************************************
* Pedestrian Detector v0.1    2013-08
*
* Fabio Reis
* [freis-at-isr.ist.utl.pt]
*
* Please email me if you find bugs, or have suggestions or questions!
* Licensed under the Simplified BSD License [see external/bsd.txt]
*******************************************************************************/
#ifndef CHNSCOMPUTE_HPP_
#define CHNSCOMPUTE_HPP_

/*
 * System includes
 */
#include <string>
#include <cstdlib>
#include <iostream>
#include <vector>

/*
 * OpenCV includes
 */
#include <opencv2/opencv.hpp>

/*
 * Include our functions
 */
#include "rgbConvertMex.hpp"
#include "rgbConvert.hpp"
#include "gradientMex.hpp"
#include "convConst.hpp"
#include "opencvInterface.hpp"

using namespace std;

enum ColorSpace {
	gray = 0,
	rgb,
	luv,
	hsv,
	orig = 1
};

enum PadWith {
	padvalue = 0,
    replicate,
    symmetric,
    circular
};

/*
 * Auxiliary function for the convTri
 */
void convTriAux(float *M, float *&S, int misalign, int height, int width,
		int chnTrans, float r, int s = 1
		);

/*
 * Class wrapping up an image
 */
class imgWrap {
public:
	float *image;

	int width;
	int height;
	int channels;

	imgWrap(float *img, int w, int h, int c) :
		image(img), width(w), height(h), channels(c) { };

};


/*
 * Class to pass the input variables
 */
class pChns {
public:
	class Color { 			//parameters for color space:
	public:
		bool enabled; 		// [1] if true enable color channels
		int colorSpace; 	// [luv] choices are: 'gray' - 0, 'rgb' - 1, 'luv' - 2, 'hsv' - 3, 'orig' - 1
	};

	class GradMag { 	//parameters for gradient magnitude:
	public:
		bool enabled; 	// [1] if true enable gradient magnitude channel
		int colorChn; 	// [0] if>0 color channel to use for grad computation
		int normRad; 	// [5] normalization radius for gradient
		float normConst; // [.005] normalization constant for gradient
	};

	class GradHist { 	// parameters for gradient histograms:
	public:
		bool enabled; 	// [1] if true enable gradient histogram channels
		int binSize; 	// [1] spatial bin size (if > 1 chns will be smaller)
		int nOrients; 	// [6] number of orientation channels
		bool softBin; 	// [0] if true use "soft" bilinear spatial binning
		bool useHog; 	// [0] if true perform 4-way hog normalization/clipping
		float clipHog;	// [.2] value at which to clip hog histogram bins
	};

	class Custom { 		// parameters for custom channels (optional struct array):
	public:
		bool enabled; 	// [1] if true enable custom channel type
		string name;	// ['REQ'] custom channel type name
		string hFunc;	// ['REQ'] function handle for computing custom channels
		void *pFunc;	// [{}] additional params for chns=hFunc(I,pFunc{:})
		int padWith;	// [0] how channel should be padded (e.g. 0,'replicate')
	};

	pChns::Color* pColor;
	pChns::GradMag* pGradMag;
	pChns::GradHist* pGradHist;
	pChns::Custom* pCustom;
	bool complete;			// [] if true does not check/set default vals in pChns



	pChns();
	~pChns();
};

/*
 * Class to output info
 */
class infoOut {
public:
	bool enableColor;
	bool enableGradMag;
	bool enableGradHist;

	int nTypes;

	pChns *input;

	int width;
	int height;

	int widthH;
	int heightH;

	imgWrap **data;
	imgWrap *I;
	imgWrap *M;
	imgWrap *H;

	//TODO : Insert Custom void* here;

	infoOut(pChns *input, int width, int height,  int _chn, int _chnTrans,
			int _widthH, int _heightH, float *_I, float *_M, float *_H
			);
};

/*
 * Compute channel features at a single scale given an input image.
 *
 * Compute the channel features as described in:
 *  P. Dollár, Z. Tu, P. Perona and S. Belongie
 *  "Integral Channel Features", BMVC 2009.
 * Channel features have proven very effective in sliding window object
 * detection, both in terms of *accuracy* and *speed*. Numerous feature
 * types including histogram of gradients (hog) can be converted into
 * channel features, and overall, channels are general and powerful.
 *
 * Given an input image I, a corresponding channel is a registered map of I,
 * where the output pixels are computed from corresponding patches of input
 * pixels (thus preserving overall image layout). A trivial channel is
 * simply the input grayscale image, likewise for a color image each color
 * channel can serve as a channel. Other channels can be computed using
 * linear or non-linear transformations of I, various choices implemented
 * here are described below. The only constraint is that channels must be
 * translationally invariant (i.e. translating the input image or the
 * resulting channels gives the same result). This allows for fast object
 * detection, as the channels can be computed once on the entire image
 * rather than separately for each overlapping detection window.
 *
 * Currently, three channel types are available by default (to date, these
 * have proven the most effective for sliding window object detection):
 *  (1) color channels (computed using rgbConvert.m)
 *  (2) gradient magnitude (computed using gradientMag.m)
 *  (3) quantized gradient channels (computed using gradientHist.m)
 * For more information about each channel type, including the exact input
 * parameters and their meanings, see the respective m-files which perform
 * the actual computatons (chnsCompute is essentially a wrapper function).
 *
 * Additionally, custom channels can be specified via an optional struct
 * array "pCustom" which may have 0 or more custom channel definitions. Each
 * custom channel is generated via a call to "chns=feval(hFunc,I,pFunc{:})".
 * The color space of I is determined by pColor.colorSpace, use the setting
 * colorSpace='orig' if the input image is not an 'rgb' image and should be
 * left unchaned (e.g. if I has multiple channels). The input I will have
 * type single and the output of hFunc should also have type single.
 *
 * As mentioned, the params for each channel type are described in detail in
 * the respective function. In addition, each channel type has a parameter
 * "enabled" that determines if the channel is computed. If chnsCompute() is
 * called with no inputs or empty I, the output is the complete default
 * parameters (pChns). Otherwise the outputs are the computed channels and
 * additional meta-data (see below). The channels are computed at a single
 * scale, for (fast) multi-scale channel computation see chnsPyramid.m.
 *
 * An emphasis has been placed on speed, with the code undergoing heavy
 * optimization. Computing the full set of channels used in the BMVC09 paper
 * referenced above on a 480x640 image runs over *100 fps* on a single core
 * of a machine from 2011 (although runtime depends on input parameters).
 *
 * USAGE
 *  chns = chnsCompute( I, pChns )
 *
 * INPUTS
 *  I           - [hxwx3] input image (uint8 or single/double in [0,1])
 *  pChns       - parameters (struct or name/value pairs)
 *   .pColor       - parameters for color space:
 *     .enabled      - [1] if true enable color channels
 *     .colorSpace   - [0] choices are: 'luv' - 0, 'gray' - 1, 'rgb' - 2, 'hsv' - 3, 'orig' - 4
 *   .pGradMag     - parameters for gradient magnitude:
 *     .enabled      - [1] if true enable gradient magnitude channel
 *     .colorChn     - [0] if>0 color channel to use for grad computation
 *     .normRad      - [5] normalization radius for gradient
 *     .normConst    - [.005] normalization constant for gradient
 *   .pGradHist    - parameters for gradient histograms:
 *     .enabled      - [1] if true enable gradient histogram channels
 *     .binSize      - [1] spatial bin size (if > 1 chns will be smaller)
 *     .nOrients     - [6] number of orientation channels
 *     .softBin      - [0] if true use "soft" bilinear spatial binning
 *     .useHog       - [0] if true perform 4-way hog normalization/clipping
 *     .clipHog      - [.2] value at which to clip hog histogram bins
 *   .pCustom      - parameters for custom channels (optional struct array):
 *     .enabled      - [1] if true enable custom channel type
 *     .name         - ['REQ'] custom channel type name
 *     .hFunc        - ['REQ'] function handle for computing custom channels
 *     .pFunc        - [{}] additional params for chns=hFunc(I,pFunc{:})
 *     .padWith      - [0] how channel should be padded (e.g. 0,'replicate')
 *   .complete     - [] if true does not check/set default vals in pChns
 *
 * OUTPUTS
 *  chns       - output struct
 *   .pChns      - exact input parameters used
 *   .nTypes     - number of channel types
 *   .data       - [nTypes x 1] cell array of channels (each is [hxwxnChns])
 *   .info       - [nTypes x 1] struct array
 *     .name       - channel type name
 *     .pChn       - exact input parameters for given channel type
 *     .nChns      - number of channels for given channel type
 *     .padWith    - how channel should be padded (0,'replicate')
 *
 * EXAMPLE
 *  I = imResample(imread('peppers.png'),[480 640]);
 *  pChns = chnsCompute(); pChns.pGradHist.binSize=4;
 *  tic, for i=1:100, chns = chnsCompute(I,pChns); end; toc
 *  figure(1); montage2(chns.data{3});
 *
 * See also rgbConvert, gradientMag, gradientHist, chnsPyramid
 *
 * Piotr's Image&Video Toolbox      Version 3.00
 * Copyright 2012 Piotr Dollar & Ron Appel.  [pdollar-at-caltech.edu]
 * Please email me if you find bugs, or have suggestions or questions!
 * Licensed under the Simplified BSD License [see external/bsd.txt]
*/

infoOut* chnsCompute(float* I, int height, int width, int channels, pChns *pchns);

#endif /* CHNSCOMPUTE_HPP_ */
