/*******************************************************************************
* Pedestrian Detector v0.1    2013-08
*
* Fabio Reis
* [freis-at-isr.ist.utl.pt]
*
* Please email me if you find bugs, or have suggestions or questions!
* Licensed under the Simplified BSD License [see external/bsd.txt]
*******************************************************************************/

#include "../include/rgbConvert.hpp"

float* rgbConvertMeta(float *image, int height, int width, int channels,
		int misalign, int colorSpace){
    int flag = colorSpace;
    float *J;

    if(flag == 4)
	flag = 1;

    bool norm=(channels==1 && flag==0) || flag==1;

    if( norm ){
	J = (float*) wrCalloc(height*width*channels+misalign,
	    sizeof(float))+ misalign;

	int lengthArray = height*width*channels;
	for (int j = 0; j < lengthArray; j++)
	    J[j] = image[j];
    }else
	J = rgbConvert(image, height*width, channels, colorSpace, 1.0f);

    return J;
}
