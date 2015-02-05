/*******************************************************************************
* Pedestrian Detector v0.1    2013-08
*
* Fabio Reis
* [freis-at-isr.ist.utl.pt]
*
* Please email me if you find bugs, or have suggestions or questions!
* Licensed under the Simplified BSD License [see external/bsd.txt]
*******************************************************************************/

#include "../include/chnsPyramid.hpp"

pyrInput::pyrInput(){
    pchns = new pChns();

    nPerOct = 8;
    nOctUp = 1;
    nApprox = -1;

    lambdas = 0;

    shrink = 4;
    pad = new int[2]{0, 0};
    minDs = new int[2]{16, 16};

    smoothIm = 1;
    smoothChns = 1;

    concat = true;
    complete = false;

    sz = new int[3]{ 0 };
}

pyrOutput* chnsPyramid(float *image, pyrInput *input){
/*
 * Declaring variables
 */
    float *I;
    int height = input->sz[0],
	    width = input->sz[1],
	    heightO = input->sz[0],
	    widthO = input->sz[1],
	    channels = input->sz[2];

    int heightOriginal = height, widthOriginal = width;
    int misalign = 1;
    int sOfF = sizeof(float);

/*
 * Get default parameters pPyramid
 */
    if ( !input->complete ){
	if(input->nApprox<0)
	    input->nApprox = input->nPerOct - 1;

	input->pchns->pGradHist->binSize = input->shrink;
	input->pad[0] = round(input->pad[0]/input->shrink) * input->shrink;
	input->pad[1] = round(input->pad[1]/input->shrink) * input->shrink;
	input->minDs[0] = max(input->minDs[0], input->shrink * 4);
	input->minDs[1] = max(input->minDs[1], input->shrink * 4);

	input->complete = true;
    }

/*
 * Convert I to appropriate color space (or simply normalize)
 */
    int cs = input->pchns->pColor->colorSpace;
    I = rgbConvert(image, height*width, channels, cs, 1.0f);
    input->pchns->pColor->colorSpace = orig;

/*
 * Get scales at which to compute features and list of real/approx scales
 */
    float *scales = 0;
    float *scaleshw = 0;
    int nScales = getScales(scales, scaleshw, input->nPerOct, input->nOctUp,
		  input->minDs, input->shrink, input->sz);
    
    //TODO :: WARNING :: This was done in the Dollar's Code
    int isRMax;
    if(true)
	isRMax = 0;
    else
	isRMax = 0 + input->nOctUp * input->nPerOct;
    //TODO :: WARNING :: END
    
    int *isR, *isA, *isN;

    //isR Vector Allocation
    int countIsR = 0;
    for ( int i = isRMax; i < nScales; i += (input->nApprox + 1) )
	countIsR++;
    
    isR = new int[countIsR];

    for ( int i = isRMax, j = 0; i < nScales; j++, i += (input->nApprox + 1) )
	isR[j] = i;

    //isA and isN Vector Allocation
    int countIsA = nScales - countIsR;
    int countIsN = nScales;
    isA = new int[countIsA];
    isN = new int[countIsN];

    for ( int i = 0, auxI = 0; i < nScales; i++){
	bool ok = true;
	for (int j = 0; j < countIsR; j++)
	    if(i == isR[j]){
		ok = false;
		break;
	    }

	if (ok){
	    isA[auxI] = i;
	    auxI++;
	}

	isN[i] = i;
    }

    int *auxIsJ;
    auxIsJ = new int[countIsR + 1];

    auxIsJ[0] = 0;
    auxIsJ[countIsR] = nScales;

    for (int i = 0; i < countIsR - 1; i++)
	auxIsJ[i+1] = floor((isR[i] + 1 + isR[i+1] + 1) / 2); 
    // " +1 " because we are in c++ and the vector already assumes the first
    // element to be index 0.

    for (int i = 0; i < countIsR; i++){
	int minJ = auxIsJ[i];
	int maxJ = auxIsJ[i+1];

	for (int j = minJ; j < maxJ; j++)
	    isN[j] = isR[i];
    }

/*
 * Compute image pyramid [real scales]
 */
    int nTypes = 0;
    imgWrap ***data =  (imgWrap ***) wrCalloc(nScales, sizeof(imgWrap**));

    int shrink = input->shrink;
    float shr[3] = { 0, 0, 0};
    for (int it = 0, i = isR[0]; it < countIsR; it++, i=isR[it]){
	float scale = scales[i];
	int newHeight = round((float) heightO * (float) scale /
			(float) shrink) * shrink;

	int newWidth = round((float) widthO * (float) scale /
			(float) shrink) * shrink;
	
	float *I1;
	if ( height == newHeight && width == newWidth){
	    //TODO :: WARNING :: Should I copy it over?
	    I1 = (float*) wrCalloc(height*width*channels + misalign,
		  sOfF) + misalign;

	    int lengthArray = height*width*channels;
	    for (int j = 0; j < lengthArray; j++)
		    I1[j] = I[j];
	}else{
	    I1 = (float*) wrCalloc(newHeight*newWidth*channels + misalign,
		  sOfF) + misalign;

	    //TODO :: WARNING :: Hardcoded value :: 1.f
	    resample(I, I1, height, newHeight, width, newWidth, channels, 1.f);
	}

	if (scale == 0.5f && (input->nApprox > 0 || input->nPerOct == 1)){
	    //TODO :: WARNING :: Should I free "I"?
	    // I replace old I with new I1, as I reduced the image to half size
	    free(I);

	    I = I1;
	    height = newHeight;
	    width = newWidth;
	}

	//TODO :: WARNING :: Hardcoded value :: downsample
	float *I2 = 0;
	int downsample = 1;
	convTriAux(I1, I2, misalign, newHeight, newWidth, channels,
		input->smoothIm, downsample
		);

	/*
	 * Channels Compute for this isR[i] scale
	 */
	infoOut *chns = chnsCompute(I2, newHeight, newWidth, channels,
			input->pchns
			);

	imgWrap **data1 = chns->data;
	wrFree(I2-misalign);
	nTypes = chns->nTypes;

	if (i == isR[0]){
	    /*
	     * This is checking the size of each transformation. By design only
	     * the H channel will have the correct dimensions.
	     */
	    for (int j = 0; j < nTypes; j++){
		shr[j] = data1[j]->height;
		shr[j] = newHeight / shr[j];

		if (shr[j] > shrink || (int)shr[j] % 1 > 0){
		    cout << "Something went wrong with the shrinking."
			<< endl << "Source code line: " << __FILE__ << " @ "
			<< __LINE__ << endl;
		    return NULL; //This should never happen
		}

		shr[j]=shr[j]/shrink;
	    }
	}

	for(int j = 0; j < nTypes; j++){
	    if (shr[j] == 1)
		continue;

	    int nH = newHeight*shr[j],
		nW = newWidth*shr[j],
		chnsTransform = data1[j]->channels;

	    float *chnTypeData = (float*) wrCalloc(nH*nW*channels + misalign,
				sOfF) + misalign;

	    //TODO :: WARNING :: Hardcoded value :: 1.f
	    resample(data1[j]->image, chnTypeData, newHeight, nH, newWidth, nW,
		    chnsTransform, 1.f
		    );

	    data1[j]->height = nH;
	    data1[j]->width = nW;

	    wrFree(data1[j]->image - misalign);
	    data1[j]->image = chnTypeData;
	}

	data[i] = data1;
    }

    // In case I changed it when scale = 0.5f
    height = heightO;
    width = widthO;

/*
 * If lambdas not specified compute image specific lambdas
 */
    int nApprox = input->nApprox;
    float *lambdas = input->lambdas;
    if ( nApprox > 0 && !lambdas){
	int nOctUp = input->nOctUp;
	int nPerOct = input->nPerOct;

	//TODO :: WARNING :: Yet again I start at 0.
	// The is Vector :: is=1 + nOctUp*nPerOct:nApprox+1:nScales;
	int countIs = 0;

	int *isTemp = new int[nScales];
	for (int i = nOctUp*nPerOct; i < nScales; i += nApprox + 1){
	    isTemp[countIs] = i;
	    countIs++;
	}

	if (countIs > 2){
	    isTemp[0] = isTemp[1];
	    isTemp[1] = isTemp[2];
	}else{
	    cout << "Couldn't calculate lambdas. Not enough scales to use."
		<< endl << "Source code line: " << __FILE__ << " @ "
		<< __LINE__ << endl;
	    return NULL;
	}


	float *f0 = new float[nTypes];
	float *f1 = new float[nTypes];

	imgWrap **d0 = data[isTemp[0]];
	imgWrap **d1 = data[isTemp[1]];

	for (int i = 0; i < nTypes; i++){
	    float numElem = (float)
			    (d0[i]->width * d0[i]->height * d0[i]->channels);

	    float sumElem = 0.f;

	    for (int j = 0; j < numElem; j++)
		sumElem += d0[i]->image[j];

	    f0[i] = sumElem / numElem;
	}

	for (int i = 0; i < nTypes; i++){
	    float numElem = (float)
			    (d1[i]->width * d1[i]->height * d1[i]->channels);

	    float sumElem = 0.f;

	    for (int j = 0; j < numElem; j++)
		sumElem += d1[i]->image[j];

	    f1[i] = sumElem / numElem;
	}


	lambdas = new float[nTypes];
	float lambdaValue = log2(scales[isTemp[0]] / scales[isTemp[1]]);
	for (int i = 0; i < nTypes; i++)
	    lambdas[i] = -log2(f0[i] / f1[i]) / lambdaValue;
    }

/*
 * Compute image pyramid [approximated scales]
 */
    shrink = input->shrink;
    for (int it = 0, i = isA[0]; it < countIsA; it++, i=isA[it]){
	int iR = isN[i];

	float scale = scales[i];

	//TODO :: WARNING :: The value height may have been modified to half of
	//                   it, hence using heightOriginal (width)
	int newHeight = round((float) heightOriginal * (float) scale /
			(float) shrink);
	int newWidth = round((float) widthOriginal * (float) scale /
			(float) shrink);

	float scaleRatio = (scale / scales[iR]);
	float *rs = new float[nTypes];

	for (int j = 0; j < nTypes; j++)
	    rs[j] = pow(scaleRatio, -lambdas[j] );

	imgWrap **dataImgA = (imgWrap **) wrCalloc(nTypes, sizeof(imgWrap*));
	imgWrap **dataImgR = data[iR];

	for (int j = 0; j < nTypes; j++){
	    int ijChannels = dataImgR[j]->channels;

	    float *isAimage = (float*) wrCalloc(newHeight*newWidth*ijChannels +
			    misalign, sOfF) + misalign;

	    //TODO :: WARNING :: Hardcoded value
	    resample(dataImgR[j]->image, isAimage, dataImgR[j]->height,
		    newHeight, dataImgR[j]->width, newWidth, ijChannels, rs[j]);

	    dataImgA[j] = new imgWrap(isAimage, newWidth, newHeight,ijChannels);
	}

	data[i] = dataImgA;
	delete [] rs;
    }

/*
 * Smooth channels, optionally pad and concatenate channels
 */
    int downSample = 1; // WARNING : This is the default by dollar
    int s = downSample;

    for (int i = 0; i < nScales; i++){
	for (int j = 0; j < nTypes; j++){
	    float *S;
	    int height = data[i][j]->height;
	    int width = data[i][j]->width;
	    int channel = data[i][j]->channels;

	    /*
	      * Smoothing Channels
	      */
	    convTriAux(data[i][j]->image, S, misalign, height, width, channel,
			input->smoothChns, s
			);

	    wrFree(data[i][j]->image - misalign);

	    /*
	     * Padding according to the scale. Then change the shrink value.
	     */
	    int padTB = input->pad[0] / shrink,
		padLR = input->pad[1] / shrink;

	    if (padTB > 0 || padLR > 0){
		int newHeight = height + padTB * 2;
		int newWidth = width + padLR * 2;

		float *P = (float*) wrCalloc(newHeight*newWidth*channel +
			    misalign, sOfF) + misalign;

		imPad(S, P, height, width, channel, padTB, padTB, padLR, padLR,
		      padvalue, 0.f
		      );

		wrFree(S - misalign);

		data[i][j]->height = newHeight;
		data[i][j]->width = newWidth;

		S = P;
	    }

	    data[i][j]->image = S;
	}
    }

/*
 * Concatenate.
 */
    int totalChannels = 0;
    for (int j = 0; j < nTypes; j++)
	totalChannels += data[0][j]->channels;

    if(input->concat){
	int chnsArr[nTypes];

	for (int j = 0; j < nTypes; j++)
	    chnsArr[j] = data[0][j]->channels;

	for (int i = 0; i < nScales; i++){
	    int height = data[i][0]->height;
	    int width = data[i][0]->width;

	    float *imgC = (float*) wrCalloc(height*width*totalChannels +
			  misalign, sOfF) + misalign;

	    int totalSize = 0;
	    for (int j = 0; j < nTypes; j++){
		float *imgO = data[i][j]->image;

		copy(imgO, imgO + height*width*chnsArr[j], imgC + totalSize);
		totalSize += height*width*chnsArr[j];
	    }

	    for (int j=1; j < nTypes; j++){
		wrFree(data[i][j]->image - misalign);
		wrFree(data[i][j]);
	    }

	    wrFree(data[i][0]->image - misalign);
	    data[i][0]->image = imgC;
	    data[i][0]->channels = totalChannels;
	}
    }


/*
 * Create output struct
 */
    pyrOutput *output = new pyrOutput();
    output->input = input;
    output->chnsPerScale = data;
    output->nScales =  nScales;
    output->scales = scales;
    output->nChannels = totalChannels;

/*
 * Clean Memory
 */
    delete [] isR;
    delete [] isA;
    delete [] isN;
    delete [] lambdas;
    delete [] scaleshw;

/*
 * Output
 */
    return output;
}

