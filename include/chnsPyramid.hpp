/*******************************************************************************
* Pedestrian Detector v0.1    2013-08
*
* Fabio Reis
* [freis-at-isr.ist.utl.pt]
*
* Please email me if you find bugs, or have suggestions or questions!
* Licensed under the Simplified BSD License [see external/bsd.txt]
*******************************************************************************/
#ifndef CHNSPYRAMID_HPP_
#define CHNSPYRAMID_HPP_

/*
 * System includes
 */
#include <cstdio>
#include <cstdlib>
#include <iostream>
#include <vector>

/*
 * Our includes
 */
#include "rgbConvertMex.hpp"
#include "getScales.hpp"
#include "imResampleMex.hpp"
#include "imPadMex.hpp"
#include "chnsCompute.hpp"

/*
 * OpenCV related includes
 */
#include "opencvInterface.hpp"
#include <opencv2/opencv.hpp>

using namespace std;
/*
 * Class to pass the input variables
 */
class pyrInput {
public:
	pChns *pchns;
	int nPerOct;	// [8] number of scales per octave
	int nOctUp;		// [0] number of upsampled octaves to compute
	int nApprox;	// [-1] number of approx. scales (if -1 nApprox=nPerOct-1)
	float* lambdas;	// [] coefficients for power law scaling (see BMVC10)
	int shrink;		// [4] integer downsampling amount for channels
	int *pad;		// [0 0] amount to pad channels (along T/B and L/R)
	int *minDs;		// [16 16] minimum image size for channel computation
	float smoothIm;	// [1] radius for image smoothing (using convTri)
	float smoothChns;// [1] radius for channel smoothing (using convTri)
	bool concat;	// [true] if true concatenate channels
	bool complete;	// [] if true does not check/set default vals in pPyramid
	int *sz;		// [] size of image H*W*C

	pyrInput();
	pyrInput(int _nPerOct, int _nOctUp, int _nApprox, float* _lambdas,
			int _shrink, int _pad[2], int _minDs[2], int _smoothIm,
			int _smoothChns, bool _concat, bool _complete, int _sz[3],
			pChns *pchns);
};

class pyrOutput {
public:
	pyrInput *input;
	imgWrap ***chnsPerScale;
	float *scales;
	int nScales;
	int nChannels;

	pyrOutput() : input(NULL), chnsPerScale(NULL), scales(NULL), nScales(0),
			nChannels(0) { };
};

/*
* Compute channel feature pyramid given an input image.
*
* While chnsCompute() computes channel features at a single scale,
* chnsPyramid() calls chnsCompute() multiple times on different scale
* images to create a scale-space pyramid of channel features.
*
* In its simplest form, chnsPyramid() first creates an image pyramid, then
* calls chnsCompute() with the specified "pChns" on each scale of the image
* pyramid. The parameter "nPerOct" determines the number of scales per
* octave in the image pyramid (an octave is the set of scales up to half of
* the initial scale), a typical value is nPerOct=8 in which case each scale
* in the pyramid is 2^(-1/8)~=.917 times the size of the previous. The
* smallest scale of the pyramid is determined by "minDs", once either image
* dimension in the resized image falls below minDs, pyramid creation stops.
* The largest scale in the pyramid is determined by "nOctUp" which
* determines the number of octaves to compute above the original scale.
*
* While calling chnsCompute() on each image scale works, it is unnecessary.
* For a broad family of features, including gradient histograms and all
* channel types tested, the feature responses computed at a single scale
* can be used to approximate feature responses at nearby scales. The
* approximation is accurate at least within an entire scale octave. For
* details and to understand why this unexpected result holds, please see:
*   P. Dollár, S. Belongie and P. Perona
*   "The Fastest Pedestrian Detector in the West," BMVC 2010.
*
* The parameter "nApprox" determines how many intermediate scales are
* approximated using the techniques described in the above paper. Roughly
* speaking, channels at approximated scales are computed by taking the
* corresponding channel at the nearest true scale (computed w chnsCompute)
* and resampling and re-normalizing it appropriately. For example, if
* nPerOct=8 and nApprox=7, then the 7 intermediate scales are approximated
* and only power of two scales are actually computed (using chnsCompute).
* The parameter "lambdas" determines how the channels are normalized (see
* the above paper). lambdas for a given set of channels can be computed
* using chnsScaling.m, alternatively, if no lambdas are specified, the
* lambdas are automatically approximated using two true image scales.
*
* Typically approximating all scales within an octave (by setting
* nApprox=nPerOct-1 or nApprox=-1) works well, and results in large speed
* gains (~4x). See example below for a visualization of the pyramid
* computed with and without the approximation. While there is a slight
* difference in the channels, during detection the approximated channels
* have been shown to be essentially as effective as the original channels.
*
* In many applications (such as object detection) the channels can be
* subsampled prior to use. Use the "shrink" parameter to subsample the
* channels by an integer amount after computation, this is also necessary
* to speed up the computation of the approximate channels described above.
*
* While every effort is made to space the image scales evenly, this is not
* always possible. For example, given a 101x100 image, it is impossible to
* downsample it by exactly 1/2 along the first dimension, moreover, the
* exact scaling along the two dimensions will differ. Instead, the scales
* are tweaked slightly (e.g. for a 101x101 image the scale would go from
* 1/2 to something like 50/101), and the output contains the exact scaling
* factors used for both the heights and the widths ("scaleshw") and also
* the approximate scale for both dimensions ("scales"). If "shrink">1 the
* scales are further tweaked so that the resized image has dimensions that
* are exactly divisible by shrink (for details please see the code).
*
* If chnsPyramid() is called with no inputs or empty I, the output is the
* complete default parameters (pPyramid). Finally, we describe the
* remaining parameters: "pad" controls the amount the channels are padded
* after being created (useful for detecting objects near boundaries);
* "smoothIm" controls the amount the image is smoothed prior to computing
* the channels (typically the amount of image smoothing should be small or
* gradient information is lost); "smoothChns" controls the amount of
* smoothing after the channels are created (and controls the integration
* scale of the channels, see the BMVC09 paper); finally "concat" determines
* whether all channels at a single scale are concatenated in the output.
*
* An emphasis has been placed on speed, with the code undergoing heavy
* optimization. Computing the full set of (approximated) *multi-scale*
* channels on a 480x640 image runs over *30 fps* on a single core of a
* machine from 2011 (although runtime depends on input parameters).
*
* USAGE
*  pyramid = chnsPyramid( I, pPyramid )
*
* INPUTS
*  I            - [hxwx3] input image (uint8 or single/double in [0,1])
*  pPyramid     - parameters (struct or name/value pairs)
*   .pChns        - parameters for creating channels (see chnsCompute.m)
*   .nPerOct      - [8] number of scales per octave
*   .nOctUp       - [0] number of upsampled octaves to compute
*   .nApprox      - [-1] number of approx. scales (if -1 nApprox=nPerOct-1)
*   .lambdas      - [] coefficients for power law scaling (see BMVC10)
*   .shrink       - [4] integer downsampling amount for channels
*   .pad          - [0 0] amount to pad channels (along T/B and L/R)
*   .minDs        - [16 16] minimum image size for channel computation
*   .smoothIm     - [1] radius for image smoothing (using convTri)
*   .smoothChns   - [1] radius for channel smoothing (using convTri)
*   .concat       - [1] if true concatenate channels
*   .complete     - [] if true does not check/set default vals in pPyramid
*
* OUTPUTS
*  pyramid      - output struct
*   .pPyramid     - exact input parameters used (may change from input)
*   .nTypes       - number of channel types
*   .nScales      - number of scales computed
*   .data         - [nScales x nTypes] cell array of computed channels
*   .info         - [nTypes x 1] struct array (mirrored from chnsCompute)
*   .lambdas      - [nTypes x 1] scaling coefficients actually used
*   .scales       - [nScales x 1] relative scales (approximate)
*   .scaleshw     - [nScales x 2] exact scales for resampling h and w
*
* EXAMPLE
*  I=imResample(imread('peppers.png'),[480 640]);
*  pPyramid=chnsPyramid(); pPyramid.minDs=[128 128];
*  pPyramid.nApprox=0; tic, P1=chnsPyramid(I,pPyramid); toc
*  pPyramid.nApprox=7; tic, P2=chnsPyramid(I,pPyramid); toc
*  figure(1); montage2(P1.data{2}); figure(2); montage2(P2.data{2});
*  figure(3); montage2(abs(P1.data{2}-P2.data{2})); colorbar;
*
* See also chnsCompute, chnsScaling, convTri, imPad
*
* Piotr's Image&Video Toolbox      Version 3.00
* Copyright 2012 Piotr Dollar & Ron Appel.  [pdollar-at-caltech.edu]
* Please email me if you find bugs, or have suggestions or questions!
* Licensed under the Simplified BSD License [see external/bsd.txt]
*/

pyrOutput* chnsPyramid(float *image, pyrInput *input);


#endif /* CHNSPYRAMID_HPP_ */
