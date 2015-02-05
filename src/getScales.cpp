/*******************************************************************************
* Pedestrian Detector v0.1    2013-08
*
* Fabio Reis
* [freis-at-isr.ist.utl.pt]
*
* Please email me if you find bugs, or have suggestions or questions!
* Licensed under the Simplified BSD License [see external/bsd.txt]
*******************************************************************************/

#include "../include/getScales.hpp"

int getScales(float *&scales, float *&scaleshw, int nPerOct, int nOctUp,
	      int* minDs, int shrink, int* sz){
    /*
     * Set each scale s such that max(abs(round(sz*s/shrink)*shrink-sz*s)) is
     * minimized without changing the smaller dim of sz (tricky algebra)
     */
    float Sz0 = (float)sz[0]/(float)minDs[0];
    float Sz1 = (float)sz[1]/(float)minDs[1];
    float minSz = (Sz0 < Sz1)? Sz0 : Sz1;

    int nScales = floor(nPerOct * (nOctUp+log2(minSz))+1);
    float scalesTemp[nScales];
    int d0, d1;

    for(int i = 0; i < nScales; i++)
	scalesTemp[i] = pow(2.f, (- ((float) i) / ((float) nPerOct)) + nOctUp);

    if (sz[0] < sz[1]){
	d0=sz[0];
	d1=sz[1];
    }else{
	d0=sz[1];
	d1=sz[0];
    }

    //WARNING :: This vectors are of size 101 in MatLab, original vector
    //		0:.01:1-eps strangely enough it goes until 1.000.
    float ss[101] = { 0 };
    float es0[101] = { 0 }, es1[101] = { 0 };
    float maxV = -1, minMaxV = -1;
    int minI;

    for (int i = 0; i < nScales; i++){
	float s = scalesTemp[i];
	float s0 = (round(d0*s/shrink)*shrink - 0.25f * shrink ) / ((float) d0);
	float s1 = (round(d0*s/shrink)*shrink + 0.25f * shrink) / ((float) d0);

	for (int j = 0; j < 101; j++) {
	    float ssj;
	    float es0j, es1j;

	    ssj = .01 * ((float) j) * (s1-s0) + s0;
	    es0j = d0 * ssj;
	    es1j = d1 * ssj;

	    ss[j] = ssj;

	    es0[j] = fabs(es0j-round(es0j/shrink)*shrink);
	    es1[j] = fabs(es1j-round(es1j/shrink)*shrink);

	    maxV = (es0[j] > es1[j])? es0[j] : es1[j];

	    if (minMaxV > maxV){
		minI = j;
		minMaxV = maxV;
	    }else if(j == 0){
		minI = j;
		minMaxV = maxV;
	    }
	}

	scalesTemp[i] = ss[minI];
    }

    int count = 0, keepScales[nScales];
    for(int i = 0; i < nScales - 1; i++){
	if(scalesTemp[i] != scalesTemp[i+1]){
	    count++;
	    keepScales[i] = true;
	}else{
	    keepScales[i] = false;
	}
    }
    count++;
    keepScales[nScales-1] = true;

    scales = new float[count];
    scaleshw = new float[count * 2];

    for(int i = 0, j = 0; i < nScales; i++){
	if(keepScales[i])
	{
	    scales[j] = scalesTemp[i];

	    scaleshw[2*j + 0] = round(sz[0]*scales[j] / shrink)*shrink / sz[0];
	    scaleshw[2*j + 1] = round(sz[1]*scales[j] / shrink)*shrink / sz[1];

	    j++;
	}
    }

    return count;
}
