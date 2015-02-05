/*******************************************************************************
* Pedestrian Detector v0.1    2013-08
*
* Fabio Reis
* [freis-at-isr.ist.utl.pt]
*
* Please email me if you find bugs, or have suggestions or questions!
* Licensed under the Simplified BSD License [see external/bsd.txt]
*******************************************************************************/

#include "../include/opencvInterface.hpp"

Mat* convertToMat(const float* data, int height, int width, int channels,
		int misalign){
    Mat* output = new Mat(Size_<int>(width, height), CV_32FC(channels));

    int x, y, c;

    if (channels == 1) {
	for(c=0; c < channels; c++)
	    for(y=0;y<height;y++)
		for(x=0;x<width;x++)
		    output->at< Vec<float, 1> >(y, x)[c] = 
			data[x*height+y + c*height*width];

    }else if (channels == 2) {
	for(c=0; c < channels; c++)
	    for(y=0;y<height;y++)
		for(x=0;x<width;x++)
		    output->at< Vec<float, 2> >(y, x)[c] =
			data[x*height+y + c*height*width];

    }else if (channels == 3) {
	for(c=0; c < channels; c++)
	    for(y=0;y<height;y++)
		for(x=0;x<width;x++)
		    output->at< Vec<float, 3> >(y, x)[c] =
			data[x*height+y + c*height*width];
    }
    
    return output;
};

Mat* convertToMatChannel(const float* data, int height, int width, int channels,
		int channel, int misalign){
    Mat* output = new Mat(Size_<int>(width, height), CV_32FC(1));

    int x, y, c = channel;
    for(y=0;y<height;y++)
	for(x=0;x<width;x++)
	    output->at< float >(y, x) = data[x*height + y + c*height*width];

    return output;
};

float* convertFromMat(const Mat& data, int height, int width, int channels,
		int misalign){
    float* output;

    int sf = sizeof(float);
    int x, y, c;
    output = (float*) wrCalloc(height*width*channels + misalign,sf) + misalign;

    for(c=0; c < channels; c++)
	for(y=0;y<height;y++)
	    for(x=0;x<width;x++)
		output[x*height+y + c*height*width] = data.at<Vec3f>(y, x)[c];

    return output;
};

void writeToMatlab(const float* data, int height, int width, int channels,
		int misalign, string filename, string name){
    std::ofstream myfile;
    myfile.open(filename.c_str());

    int x, y, c;

    for(c=0; c < channels; c++){
	myfile << name << "(:,:," << c + 1 << ") = [";
	for(y=0;y<height;y++){
	    for(x=0;x<width;x++)
		myfile << data[x*height+y + c*height*width]<< " ";
	    myfile << ";" << std::endl;
	}
	myfile << "];" << std::endl;
    }

    myfile.close();
}

void writeToConsole(const float* data, int height, int width, int channels,
		int misalign){
    int x, y, c;

    for(c=0; c < channels; c++){
	std::cout << "cppImage" << c << " = [";
	for(y=0;y<height;y++){
	    for(x=0;x<width;x++)
		std::cout << data[x*height+y + c*height*width]<< " ";
	    std::cout << ";" << std::endl;
	}
	std::cout << "];" << std::endl;
    }
}
