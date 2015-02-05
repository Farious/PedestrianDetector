/*******************************************************************************
* Pedestrian Detector v0.1    2013-08
*
* Fabio Reis and Matteo Taiana
* [freis-at-isr.ist.utl.pt] and [mtaiana-at-isr.ist.utl.pt]
* 
* Original code by Matteo Taiana targeted for Matlab.
*
* Please email us if you find bugs, or have suggestions or questions!
* Licensed under the Simplified BSD License [see external/bsd.txt]
*******************************************************************************/

#include "../include/strongClassifierTree.hpp"

double *classifierData  = NULL; //global variable, accessible from everywhere
int nWeakClassifiers = 0;

/*
 * Classifier Input constructor
 */
classifierInput::classifierInput(ClassData *classifier,
                                 ClassRectangles *rect,
                                 bool _verbose,
                                 float _widthOverHeight,
                                 int _shrinkFactor,
                                 int _theoreticalWindowWidth,
                                 int _theoreticalWindowHeight,
                                 float _theoreticalActiveWindowWidth,
                                 float _theoreticalActiveWindowHeight,
                                 int _nBaseFeatures,
                                 int _nExtraFeatures
                                 ){
    classData = classifier;
    classRect = rect;

    widthOverHeight = _widthOverHeight;
    shrinkFactor = _shrinkFactor;

    theoreticalWindowWidth = _theoreticalWindowWidth;
    theoreticalWindowHeight = _theoreticalWindowHeight;
    theoreticalActiveWindowWidth  = _theoreticalActiveWindowWidth;
    theoreticalActiveWindowHeight = _theoreticalActiveWindowHeight;

    windowWidth = floor( theoreticalWindowWidth  / shrinkFactor );
    windowHeight = floor( theoreticalWindowHeight / shrinkFactor );

    activeWindowWidth  =  floor( theoreticalActiveWindowWidth  / shrinkFactor );
    activeWindowHeight =  floor( theoreticalActiveWindowHeight / shrinkFactor );

    windowHorizontalPadding = ceil((windowWidth - activeWindowWidth) / 2);
    windowVerticalPadding   = ceil((windowHeight - activeWindowHeight) / 2);

    theoreticalHorizontalPadding = ceil((theoreticalWindowWidth - theoreticalActiveWindowWidth) / 2);
    theoreticalVerticalPadding   = ceil((theoreticalWindowHeight - theoreticalActiveWindowHeight) / 2);


    imageHorizontalPadding	= windowHorizontalPadding + 1;
    imageVerticalPadding	= windowVerticalPadding + 1;
    verticalSuperPadding	= 0;
    horizontalSuperPadding	= 0;

    returnFeatures	= true;
    returnVotes		= false;
    verbose			= _verbose;
    performNMS		= true;

    // 1. Get data for classifierData
    nClassifiers = classData->nRows;
    nWeakClassifiers = classData->nCols;
    nFeatures = classRect->nRows;
    nRectRows = classRect->nRows;
    nRectCols = classRect->nCols;
    
    // This values?
    nBaseFeatures = _nBaseFeatures;
    nExtraFeatures = _nExtraFeatures;
}


/*
 * Detection Structure for output
 */
struct Detection
{
    double U0;
    double V0;
    double U1;
    double V1;
    double confidence;
};

/*
 * Function to compare Detections
 */
bool compareDetections(Detection d1, Detection d2){
    return (d1.confidence>d2.confidence);
}

// Functions to access classifier data, please take care that these pretain 
// directly to the rectangles.dat file used! Where there are 14 cols.
double alpha(int row){
    int col=13;
    return classifierData[row*nWeakClassifiers+ col];
}

int feature(int row){
    int col=1;
    return classifierData[row*nWeakClassifiers+ col];
}

double threshold(int row){
    int col=2;
    return classifierData[row*nWeakClassifiers+ col];
}

int direction(int row){
    int col=3;
    return classifierData[row*nWeakClassifiers+ col];
}

int featureSat(int row){
    int col=5;
    return classifierData[row*nWeakClassifiers+ col];
}

double thresholdSat(int row){
    int col=6;
    return classifierData[row*nWeakClassifiers+ col];
}

int directionSat(int row){
    int col=7;
    return classifierData[row*nWeakClassifiers+ col];
}

int featureNotSat(int row){
    int col=9;
    return classifierData[row*nWeakClassifiers+ col];
}

double thresholdNotSat(int row){
    int col=10;
    return classifierData[row*nWeakClassifiers+ col];
}

int directionNotSat(int row){
    int col=11;
    return classifierData[row*nWeakClassifiers+ col];
}


/*
 * Strong Classifier Tree (sct)
 */
vector<cv::Rect_<int> >* sctRun(pyrOutput *outputPyr, classifierInput *cInput)
{
    /*
     * Input explicit variables declaration
     */
    imgWrap ***pyrData = outputPyr->chnsPerScale;

    bool verbose = cInput->verbose;
    bool returnFeatures = cInput->returnFeatures;
    bool returnVotes = cInput->returnVotes;

    classifierData = cInput->classData->classifiers;
    int *rectangles =  cInput->classRect->rectangles;
    
    nWeakClassifiers = cInput->nWeakClassifiers;
    int nFeatures = cInput->nFeatures;
    int nClassifiers = cInput->nClassifiers;
    int nBaseFeatures = cInput->nBaseFeatures;
    int nExtraFeatures =  cInput->nExtraFeatures;
    int nRectRows = cInput->nRectRows;
    int nRectCols = cInput->nRectCols;
    
    int windowWidth = cInput->windowWidth;
    int windowHeight = cInput->windowHeight;
    float theoreticalActiveWindowWidth = cInput->theoreticalActiveWindowWidth;
    float theoreticalActiveWindowHeight = cInput->theoreticalActiveWindowHeight;
    int horizontalSuperPadding = cInput->horizontalSuperPadding;
    int verticalSuperPadding = cInput->verticalSuperPadding;

    int nScales = outputPyr->nScales;
    int nChannels = outputPyr->nChannels;
    float *scales = outputPyr->scales;


    /*
     * Variables declaration
     */
    int row = 0;
    int col = 0;
    int scaleId = -1;
    int classifierId = -1;
    int nDetections = 0;
    float *IC = NULL;
    float *data;
    int nRows;
    int nCols;
    imgWrap *currentScaleData;


    //DEBUG
    if(verbose){
	cout<<"**********************************************************"<<endl;
	cout<<"Initialization:"<<endl;
	cout<<"nFeatures      = "<<nFeatures   <<endl;
	cout<<"nScales        = "<<nScales     <<endl;
	cout<<"windowWidth    = "<<windowWidth <<endl;
	cout<<"windowHeight   = "<<windowHeight<<endl;
	cout<<"theoreticalActiveWindowWidth  = "<<theoreticalActiveWindowWidth<<endl;
	cout<<"theoreticalActiveWindowHeight = "<<theoreticalActiveWindowHeight<<endl;
	cout<<"windowHorizontalPadding  = "<<cInput->windowHorizontalPadding<<endl;
	cout<<"windowVerticalPadding = "<<cInput->windowVerticalPadding<<endl;
	cout<<"returnFeatures = "<< returnFeatures<<endl;
	cout<<"returnVotes    = "<< returnVotes<<endl;
	cout<<"nClassifiers   = "<< nClassifiers<<endl;
	cout<<"**********************************************************"<<endl;
    }

    //Detections to ouput
    list<Detection> detections;
    Detection currentDetection;

    //Features to output, if needed
    list< double* > ChnFtrs;
    double *currentChnFtr = NULL;

    //Votes to output, if needed
    list< double* > Votes;
    double *currentVotes = NULL;
    if(returnVotes) currentVotes = new double [nClassifiers];

    /*
     * Begin cycling through all the scales and running the detector on them.
     */
    for(scaleId=0; scaleId<nScales; scaleId++){
	//For all scales
	//Get size of the images for this size and the pointer to the data

	//All the channels are concatenated we get the first and only imgWrap.
	currentScaleData = pyrData[scaleId][0];
	data = currentScaleData->image; //Get the pointer the data
	nRows = currentScaleData->height;
	nCols = currentScaleData->width;
	
	//Only detect when the image is bigger than the detection window
	if((nRows < windowHeight) || (nCols < windowWidth))
	    continue; //skip this size: it's too small to use our detector

	// 1.1 Allocate memory for the Integral Channels of this scale
	IC = new float [nChannels * (nRows+1) * (nCols+1)];

	//**************************************************************
	// 1. Build the Integral Channels for this layer of the pyramid
	//**************************************************************
	int channelId;
	for( channelId = 0; channelId < nChannels; channelId++ ){
	    // 1.2 Build the Integral Channel for the current channel

	    // Fill the first column with zeros. WARNING Could I use memset() on
	    // floats?
	    col = 0;
	    for(row = 0; row<nRows+1; row++)
		    IC[channelId*(nRows+1)*(nCols+1) + col*(nRows+1) + row] = 0;
	    // WARNING col is there just for clarity. If I remove it, does the
	    // code get faster?

	    for(col = 1; col<nCols+1; col++){
		//set one variable in the first row to 0
		row = 0;
		IC[channelId*(nRows+1)*(nCols+1) + col*(nRows+1) + row] = 0;
		// WARNING row is there just for clarity. If I remove it,
		// does the code get faster?

		float cumulative = 0;
		// Holds the sum of the elements of the current column, up to the
		// current row.

		for(row = 1; row<nRows+1; row++){
			cumulative += data[channelId*(nRows)*(nCols) + (col-1)*(nRows) + row-1];

			IC[channelId*(nRows+1)*(nCols+1) + col*(nRows+1) + row] = cumulative
					+ IC[channelId*(nRows+1)*(nCols+1) + (col-1)*(nRows+1) + row];
		}
	    }
	}

	//*************************************
	// 2. Run the soft cascade on the data
	//*************************************
	int windowCounter=0;
	double weakClass = 0;
	for (col = -1; col<(nCols - windowWidth); col++ ){
	    for (row = -1; row<(nRows - windowHeight); row++ ){
		//Run the detector on this window
		double confidence = 0;
		for (classifierId = 0; classifierId<nClassifiers; classifierId++){

		    // Compute the value of the 3 features associated with this
		    // weak classifier
		    // 1. Root
		    int featureId = feature(classifierId);
		    double thresholdValue = threshold(classifierId);
		    double directionValue = direction(classifierId);
		    double featureValue;

		    if(featureId<nBaseFeatures){ //if feature 0<=f<=2999
			    // WARNING: wrong way of accessing the data: lots of
			    // cache faults. The data is in consecutive columns,
			    // but the memory is contiguous along the rows. I should
			    // transpose the matrix to solve this problem.

			    //-1 beacuse of matlab/c++ conventions
			    int channelId = rectangles[featureId*nRectCols + 4] -1;

			    int row0    = rectangles[featureId*nRectCols + 0]+row;
			    int col0    = rectangles[featureId*nRectCols + 1]+col;
			    int rowEnd  = rectangles[featureId*nRectCols + 2]+row;
			    int colEnd  = rectangles[featureId*nRectCols + 3]+col;
			    featureValue =
			        + IC[channelId*(nRows+1)*(nCols+1) + (colEnd+1) *(nRows+1) + (rowEnd+1)]
			        - IC[channelId*(nRows+1)*(nCols+1) + (col0)     *(nRows+1) + (rowEnd+1)]
			        - IC[channelId*(nRows+1)*(nCols+1) + (colEnd+1) *(nRows+1) + (row0)    ]
			        + IC[channelId*(nRows+1)*(nCols+1) + (col0)     *(nRows+1) + (row0)    ];
		        }else{
			    //3000 - 3000  = channel 0
			    int channelId = featureId - nBaseFeatures;
			    double outside = 0;
			    double inside = 0;
			    int c= col +1; //this way it will go from 0 to...
			    int r= row +1; //this way it will go from 0 to...

			    outside = + IC[channelId*(nRows+1)*(nCols+1) + (12+c) *(nRows+1) + (31 +r)] //A
				      - IC[channelId*(nRows+1)*(nCols+1) + ( 8+c) *(nRows+1) + (27 +r)] //B
				      + IC[channelId*(nRows+1)*(nCols+1) + ( 8+c) *(nRows+1) + ( 3 +r)] //C
				      + IC[channelId*(nRows+1)*(nCols+1) + ( 3+c) *(nRows+1) + (27 +r)] //D
				      - IC[channelId*(nRows+1)*(nCols+1) + ( 3+c) *(nRows+1) + ( 3 +r)] //E
				      - IC[channelId*(nRows+1)*(nCols+1) + (12+c) *(nRows+1) + ( 0 +r)] //F
				      - IC[channelId*(nRows+1)*(nCols+1) + ( 0+c) *(nRows+1) + (31 +r)] //G
				      + IC[channelId*(nRows+1)*(nCols+1) + ( 0+c) *(nRows+1) + ( 0 +r)];//H

			    inside  = + IC[channelId*(nRows+1)*(nCols+1) + ( 8+c) *(nRows+1) + (27 +r)] //B
				      - IC[channelId*(nRows+1)*(nCols+1) + ( 8+c) *(nRows+1) + ( 3 +r)] //C
				      - IC[channelId*(nRows+1)*(nCols+1) + ( 3+c) *(nRows+1) + (27 +r)] //D
				      + IC[channelId*(nRows+1)*(nCols+1) + ( 3+c) *(nRows+1) + ( 3 +r)];//E

			    //featureValue = outside - inside; //TODO :: I added this line - Fabio
		        }

		        if( (featureValue - thresholdValue) * directionValue >=0 ){
			    // 2. Satisfy leaf
			    // I removed the +1 because the id's are in C++ style,
			    // they start from 0
			    int featureIdSat         = featureSat(  classifierId);
			    double thresholdValueSat = thresholdSat(classifierId);
			    double directionValueSat = directionSat(classifierId);
			    double featureValueSat;

			    if(featureIdSat<nBaseFeatures){ 
			        //if feature 0<=f<=2999
			        // WARNING: wrong way of accessing the data: lots of
			        // cache faults. The data is in consecutive columns,
			        // but the memory is contiguous along the rows.
			        // I should transpose the matrix to solve this problem.

			        //-1 beacuse of matlab/c++ conventions
			        int channelId = rectangles[featureIdSat*nRectCols + 4] -1;
			        int row0    = rectangles[featureIdSat*nRectCols + 0]+row;
			        int col0    = rectangles[featureIdSat*nRectCols + 1]+col;
			        int rowEnd  = rectangles[featureIdSat*nRectCols + 2]+row;
			        int colEnd  = rectangles[featureIdSat*nRectCols + 3]+col;

			        featureValueSat = + IC[channelId*(nRows+1)*(nCols+1) + (colEnd+1) *(nRows+1) + (rowEnd+1)]
					          - IC[channelId*(nRows+1)*(nCols+1) + (col0)     *(nRows+1) + (rowEnd+1)]
					          - IC[channelId*(nRows+1)*(nCols+1) + (colEnd+1) *(nRows+1) + (row0)    ]
					          + IC[channelId*(nRows+1)*(nCols+1) + (col0)     *(nRows+1) + (row0)    ];
			    }else{
			        //3000 - 3000  = channel 0
			        int channelId = featureIdSat - nBaseFeatures ;

			        if((channelId<0)||(channelId>9)){
				    cout <<"error: negative channel\n";
				    cout << "Negative channel." << endl
				          << "Source code line: " << __FILE__
				          << " @ " << __LINE__ << endl;
				    return NULL;
			        }

			        double outside = 0;
			        double inside = 0;
			        int c= col +1; //this way it will go from 0 to...
			        int r= row +1; //this way it will go from 0 to...

			        outside = + IC[channelId*(nRows+1)*(nCols+1) + (12+c) *(nRows+1) + (31 +r)] //A
				          - IC[channelId*(nRows+1)*(nCols+1) + ( 8+c) *(nRows+1) + (27 +r)] //B
				          + IC[channelId*(nRows+1)*(nCols+1) + ( 8+c) *(nRows+1) + ( 3 +r)] //C
				          + IC[channelId*(nRows+1)*(nCols+1) + ( 3+c) *(nRows+1) + (27 +r)] //D
				          - IC[channelId*(nRows+1)*(nCols+1) + ( 3+c) *(nRows+1) + ( 3 +r)] //E
				          - IC[channelId*(nRows+1)*(nCols+1) + (12+c) *(nRows+1) + ( 0 +r)] //F
				          - IC[channelId*(nRows+1)*(nCols+1) + ( 0+c) *(nRows+1) + (31 +r)] //G
				          + IC[channelId*(nRows+1)*(nCols+1) + ( 0+c) *(nRows+1) + ( 0 +r)];//H

			        inside  = + IC[channelId*(nRows+1)*(nCols+1) + ( 8+c) *(nRows+1) + (27 +r)] //B
				          - IC[channelId*(nRows+1)*(nCols+1) + ( 8+c) *(nRows+1) + ( 3 +r)] //C
				          - IC[channelId*(nRows+1)*(nCols+1) + ( 3+c) *(nRows+1) + (27 +r)] //D
				          + IC[channelId*(nRows+1)*(nCols+1) + ( 3+c) *(nRows+1) + ( 3 +r)];//E

			        featureValueSat = outside - inside;
			    }

			    if( (featureValueSat - thresholdValueSat) * directionValueSat >=0 )
			        weakClass =  1;
			    else
			        weakClass = -1;
		    }else{
			// 3. Not Satisfy leaf
			// I removed the +1 because the id's are in C++ style,
			// they start from 0
			int featureIdNotSat         = featureNotSat(  classifierId);
			double thresholdValueNotSat = thresholdNotSat(classifierId);
			double directionValueNotSat = directionNotSat(classifierId);
			double featureValueNotSat;

			if(featureIdNotSat<nBaseFeatures){ 
			    //if feature 0<=f<=2999
			    // WARNING: wrong way of accessing the data: lots of
			    // cache faults. The data is in consecutive columns,
			    // but the memory is contiguous along the rows.
			    // I should transpose the matrix to solve this problem.

			    //-1 beacuse of matlab/c++ conventions
			    int channelId = rectangles[featureIdNotSat*nRectCols + 4] -1;
			    int row0    = rectangles[featureIdNotSat*nRectCols + 0]+row;
			    int col0    = rectangles[featureIdNotSat*nRectCols + 1]+col;
			    int rowEnd  = rectangles[featureIdNotSat*nRectCols + 2]+row;
			    int colEnd  = rectangles[featureIdNotSat*nRectCols + 3]+col;
			    featureValueNotSat =  + IC[channelId*(nRows+1)*(nCols+1) + (colEnd+1) *(nRows+1) + (rowEnd+1)]
						  - IC[channelId*(nRows+1)*(nCols+1) + (col0)     *(nRows+1) + (rowEnd+1)]
						  - IC[channelId*(nRows+1)*(nCols+1) + (colEnd+1) *(nRows+1) + (row0)    ]
						  + IC[channelId*(nRows+1)*(nCols+1) + (col0)     *(nRows+1) + (row0)    ];
			}else{
			    //3000 - 3000 = channel 0
			    int channelId = featureIdNotSat - nBaseFeatures;

			    if((channelId<0)||(channelId>9)){
				cout <<"error: negative channel\n";
				cout << "Negative channel." << endl
				      << "Source code line: " << __FILE__
				      << " @ " << __LINE__ << endl;
				return NULL;
			    }

			    double outside = 0;
			    double inside = 0;
			    int c= col +1; //this way it will go from 0 to...
			    int r= row +1; //this way it will go from 0 to...
			    outside = + IC[channelId*(nRows+1)*(nCols+1) + (12+c) *(nRows+1) + (31 +r)] //A
				      - IC[channelId*(nRows+1)*(nCols+1) + ( 8+c) *(nRows+1) + (27 +r)] //B
				      + IC[channelId*(nRows+1)*(nCols+1) + ( 8+c) *(nRows+1) + ( 3 +r)] //C
				      + IC[channelId*(nRows+1)*(nCols+1) + ( 3+c) *(nRows+1) + (27 +r)] //D
				      - IC[channelId*(nRows+1)*(nCols+1) + ( 3+c) *(nRows+1) + ( 3 +r)] //E
				      - IC[channelId*(nRows+1)*(nCols+1) + (12+c) *(nRows+1) + ( 0 +r)] //F
				      - IC[channelId*(nRows+1)*(nCols+1) + ( 0+c) *(nRows+1) + (31 +r)] //G
				      + IC[channelId*(nRows+1)*(nCols+1) + ( 0+c) *(nRows+1) + ( 0 +r)];//H

			    inside  = + IC[channelId*(nRows+1)*(nCols+1) + ( 8+c) *(nRows+1) + (27 +r)] //B
				      - IC[channelId*(nRows+1)*(nCols+1) + ( 8+c) *(nRows+1) + ( 3 +r)] //C
				      - IC[channelId*(nRows+1)*(nCols+1) + ( 3+c) *(nRows+1) + (27 +r)] //D
				      + IC[channelId*(nRows+1)*(nCols+1) + ( 3+c) *(nRows+1) + ( 3 +r)];//E

			    featureValueNotSat = outside - inside;
			}

			if( (featureValueNotSat - thresholdValueNotSat) * directionValueNotSat >=0 )
			    weakClass =  1;
			else
			    weakClass = -1;
		    }

		    confidence = confidence + weakClass*alpha(classifierId);

		    //store it to pass it back to Matlab
		    if(returnVotes)
			currentVotes[classifierId]=weakClass;

		    //WARNING Euristic value from Dollar
		    if(confidence < magicThreshold)
			break;
		} //For each classifier

		if(confidence>0){
		    nDetections++;

		    //Pre - BMVC
		    //*/ col and row are the coordinates in the shrinked, padded image
		    double V0 = ( (double)(row+1) *4  - verticalSuperPadding) / scales[scaleId] ; //both window and image are padded: this takes care of itself, I don't have to bother
		    double U0 = ( (double)(col+1) *4  - horizontalSuperPadding) / scales[scaleId] ; //both window and image are padded: this takes care of itself, I don't have to bother
		    double V1 = V0 + ( theoreticalActiveWindowHeight / scales[scaleId] );
		    double U1 = U0 + ( theoreticalActiveWindowWidth  / scales[scaleId] );
		    //detections(nDetections,:) = [V0,U0,V1,U1,confidence];
		    //*/

		    /*/BMVC
		    double windowHorPad = 2; //WARNING HARDCODED value
		    double windowVerPad = 4; //WARNING HARDCODED value
		    double imageHorPad = cInput->imageHorizontalPadding; //3
		    double imageVerPad = cInput->imageVerticalPadding; //6
		    double horPadDiff= imageHorPad - windowHorPad ;
		    double verPadDiff= imageVerPad - windowVerPad ;
		    // col and row are the coordinates in the shrinked, padded image
		    double V0 = ( ( (double)(row +1 -verPadDiff) *4 ) / scales[scaleId] ) ;   //both window and image are padded: this takes care of itself, I don't have to bother
		    double U0 = ( ( (double)(col +1 -horPadDiff) *4 ) / scales[scaleId] ) ; //both window and image are padded: this takes care of itself, I don't have to bother
		    double V1 = V0 + ( theoreticalActiveWindowHeight / scales[scaleId] );
		    double U1 = U0 + ( theoreticalActiveWindowWidth  / scales[scaleId] );
		    //*/
		    if(verbose)
			printf("Detection #%d. Scale = %d, scaling = %f, "
			    "Col = %d, Row = %d, U0 = %f, V0 = %f\n",
			    nDetections, scaleId, scales[scaleId],
			    col, row, U0, V0
			    );

		    //Add the bounding box + confidence to the results that will be returned
		    currentDetection.V0         = V0;
		    currentDetection.V1         = V1;
		    currentDetection.U0         = U0;
		    currentDetection.U1         = U1;
		    currentDetection.confidence = confidence;
		    detections.push_back(currentDetection);

		    if(returnFeatures){
			currentChnFtr = new double [nBaseFeatures+nExtraFeatures];
			int featureId;
			//nBaseFeatures
			for( featureId = 0; featureId<nBaseFeatures; featureId++ )
				currentChnFtr[featureId]=-5;

			for( featureId = 0; featureId<nBaseFeatures; featureId++ ){
			    //WARNING: wrong way of accessing the data: lots of cache faults. The data is in consecutive columns, but the memory is contiguous along the rows. I should transpose the matrix to solve this problem.
			    int channelId = rectangles[featureId*nRectCols + 4] -1; //-1 beacuse of matlab/c++ conventions
			    int row0    = rectangles[featureId*nRectCols + 0]+row;
			    int col0    = rectangles[featureId*nRectCols + 1]+col;
			    int rowEnd  = rectangles[featureId*nRectCols + 2]+row;
			    int colEnd  = rectangles[featureId*nRectCols + 3]+col;

			    currentChnFtr[featureId] = + IC[channelId*(nRows+1)*(nCols+1) + (colEnd+1) *(nRows+1) + (rowEnd+1)]
							- IC[channelId*(nRows+1)*(nCols+1) + (col0)     *(nRows+1) + (rowEnd+1)]
							- IC[channelId*(nRows+1)*(nCols+1) + (colEnd+1) *(nRows+1) + (row0)    ]
							+ IC[channelId*(nRows+1)*(nCols+1) + (col0)     *(nRows+1) + (row0)    ];
			}

			int extraFeatureId;
			for(extraFeatureId = 0; extraFeatureId<10; extraFeatureId++){
			    //REMOVED FOR NOW, IT'S NOT BEING USED.
			    //                 double outside = 0;
			    //                 double inside = 0;
			    //                 int c= col +1; //this way it will go from 0 to...
			    //                 int r= row +1; //this way it will go from 0 to...
			    //                 outside = + IC[extraFeatureId*(nRows+1)*(nCols+1) + (12+c) *(nRows+1) + (31 +r)] //A
			    //                           - IC[extraFeatureId*(nRows+1)*(nCols+1) + ( 8+c) *(nRows+1) + (27 +r)] //B
			    //                           + IC[extraFeatureId*(nRows+1)*(nCols+1) + ( 8+c) *(nRows+1) + ( 3 +r)] //C
			    //                           + IC[extraFeatureId*(nRows+1)*(nCols+1) + ( 3+c) *(nRows+1) + (27 +r)] //D
			    //                           - IC[extraFeatureId*(nRows+1)*(nCols+1) + ( 3+c) *(nRows+1) + ( 3 +r)] //E
			    //                           - IC[extraFeatureId*(nRows+1)*(nCols+1) + (12+c) *(nRows+1) + ( 0 +r)] //F
			    //                           - IC[extraFeatureId*(nRows+1)*(nCols+1) + ( 0+c) *(nRows+1) + (31 +r)] //G
			    //                           + IC[extraFeatureId*(nRows+1)*(nCols+1) + ( 0+c) *(nRows+1) + ( 0 +r)];//H
			    //
			    //                 inside  = + IC[extraFeatureId*(nRows+1)*(nCols+1) + ( 8+c) *(nRows+1) + (27 +r)] //B
			    //                           - IC[extraFeatureId*(nRows+1)*(nCols+1) + ( 8+c) *(nRows+1) + ( 3 +r)] //C
			    //                           - IC[extraFeatureId*(nRows+1)*(nCols+1) + ( 3+c) *(nRows+1) + (27 +r)] //D
			    //                           + IC[extraFeatureId*(nRows+1)*(nCols+1) + ( 3+c) *(nRows+1) + ( 3 +r)];//E
			    //
			    //                 currentChnFtr[nBaseFeatures +extraFeatureId ] = outside - inside;
			    currentChnFtr[nBaseFeatures+extraFeatureId]=-555;//TODO :: No Hardcoded
			}

			    ChnFtrs.push_back(currentChnFtr);
		    }

		    if(returnVotes){
			//push the votes for the current example on the list
			Votes.push_back(currentVotes);
			//allocate space for the votes of the next positive example
			currentVotes = new double [nClassifiers];
		    }
		} //else do nothing, discard the window
	    } //scan image rows
	} //scan image columns

	delete[] IC;
	IC = NULL;
    }

    //*************************************
    // 2. Non-Maximal Suppression Part
    //*************************************

    //NMS code ported from some Matlab code by Piotr Dollár, see his Matlab toolbox for indications on which papers to cite, if you decide to use this code.
    bool performNms = true;
    bool greedy = true;
    bool ovrDnm = false;
    double overlap = 0.65;

    if(nDetections>0){
	if(performNms){
	    //Sort the "detections" structure according to the confidence value, in descending order
	    detections.sort(compareDetections);

	    //Initialize the keep vector
	    vector<bool> keep(nDetections, true); //initialize all keep elements to true.

	    //Compute area of each BB
	    vector<double> areas(nDetections);
	    list<Detection>::iterator itDetections;
	    vector<double>::iterator itV;
	    itV=areas.begin();
	    for(itDetections=detections.begin(); itDetections!=detections.end(); ++itDetections){
		*itV = ( (*itDetections).U1 - (*itDetections).U0 ) * ( (*itDetections).V1 - (*itDetections).V0 );
		itV++;
	    }

	    vector<Detection>detVector(detections.begin(), detections.end()); //copy the content of the list of detections to a vector

	    for(int i=0; i<nDetections; i++){
		if(greedy && keep[i]==false) continue; //If this bb has already been discarded, let's ignore it and skip to the next

		for(int j=i+1; j<nDetections; j++){
		    if(keep[j]==false) continue; //No need to compare with the previously discarded examples
		    double iw = min(detVector[i].U1, detVector[j].U1 ) - max(detVector[i].U0, detVector[j].U0);
		    if(iw<=0) continue; //No horizontal overlap -> no need to compare these two bb's
		    double ih = min(detVector[i].V1, detVector[j].V1 ) - max(detVector[i].V0, detVector[j].V0); ;
		    if(ih<=0) continue; //No vertical overlap -> no need to compare these two bb's
		    double o = iw*ih; //Compute overlap area
		    double u;
		    if(ovrDnm)
			u = areas[i]+areas[j]-o; //Denominator of the fraction  = union of bb's "i" and "j", PASCAL rule
		    else
			u = min(areas[i],areas[j]); //Denominator of the fraction = smallest area between "i" and "j"

		    o = o/u; //Compute the ratio of the areas
		    if(o>overlap) //When two bb's match, suppress the least confident one
			keep[j]=false;
		}
	    }

	    //Remove the elements with keep() == false from the list
	    vector<bool>::iterator itKeep;
	    itDetections=detections.begin();
	    for(itKeep = keep.begin(); itKeep!=keep.end(); ++itKeep){
		//cout<<(*itDetections).confidence<<" ";
		if((*itKeep)==false){
		    //cout<<"Erasing!\n";
		    itDetections = detections.erase(itDetections);
		}else{
		    //cout<<"Keeping.\n";
		    itDetections++;
		}
	    }

	    nDetections = detections.size();
	    if(verbose)
  	    cout<<"nDetections="<<nDetections<<endl;
	}
    }
    
    vector<cv::Rect_<int> > *listRect = new vector<cv::Rect_<int> >();
    list<Detection>::iterator itDetections;
    for(itDetections=detections.begin(); itDetections!=detections.end(); ++itDetections){
        if(verbose)
          cout << itDetections->confidence 
               << ", (" << itDetections->U0 
               << ", " << itDetections->V0 
               << ") x (" 
               <<  itDetections->U1 
               << ", " 
               << itDetections->V1 
               << ")" << endl;
               
        int ax = itDetections->U0;
        int ay = itDetections->V0;
        int aw = itDetections->U1 - ax;
        int ah = itDetections->V1 - ay;
        listRect->push_back(Rect_<int>(ax, ay, aw, ah));
    }
  
    
    return listRect;
}
