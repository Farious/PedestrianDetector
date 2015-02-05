/*******************************************************************************
* Pedestrian Detector v0.1    2013-08
*
* Fabio Reis
* [freis-at-isr.ist.utl.pt]
*
* Please email me if you find bugs, or have suggestions or questions!
* Licensed under the Simplified BSD License [see external/bsd.txt]
*******************************************************************************/

#include <cstdio>
#include <cstdlib>
#include <iostream>
#include <algorithm>

/*
 * Our detector
 */
#include "../include/pedDetector.hpp"

/*
 * OpenCV Includes
 */
#include <opencv2/opencv.hpp>

using namespace cv;
using namespace std;

/*
 * Output help summary
 */
void help(){
  

}

/*
 * Input Parsing
 */
int parseInput(int argc, char *argv[], string &filename){
  string fname;
  if(argc == 1){
    fname = string(argv[1]);
    filename = fname;
    return 1;
  }else if(argc > 2)
    return 0;
    
  string inToggle = string(argv[1]);
  transform(inToggle.begin(), inToggle.end(), inToggle.begin(), ::tolower);
  if(inToggle.compare("-f") != 0){
    cout << "Wrong input" << endl;
    return 0;
  }else{
    fname = string(argv[2]);
    filename = fname;
    return 1;
  }
}

int main(int argc, char *argv[]){
  string filename;
  if(!parseInput(argc - 1, argv, filename))
    return 0;
  
	Mat image = imread(filename.c_str());
	
	/*
	 * Running the detector
	 */
	vector<cv::Rect_<int> >* rects = pedDetector(image);

	/*
	 * OpenCV Output
	 */
	/*/
	namedWindow("Teste", CV_WINDOW_NORMAL);
	vector<cv::Rect_<int> >::iterator it;
	for(it = rects->begin(); it != rects->end(); it++)
		rectangle(image, *it, Scalar_<int>(255,0,0));
	imshow("Teste", image);
	waitKey(0);
  //*/
  
	/*
	 * File Output
	 */
  string resultTXT = filename.substr(0,filename.length()-1-3) + ".txt";
	ofstream myfile;
	myfile.open(resultTXT.c_str());
  myfile << "Each line of this file is of the type:" << endl 
         << "ID : X, Y, Height, Width" << endl;
  
	vector<cv::Rect_<int> >::iterator it;
	int detection = 0;
	for(it = rects->begin(); it != rects->end(); it++, detection++){
	  myfile << detection << " : " << it->x << ", " << it->y << ", " << 
	                                  it->height << ", " << it->width << endl;
	}
	myfile.close();


	/*
	 * Free Memory
	 */
	delete(rects);
	return 0;
}
