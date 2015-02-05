/*******************************************************************************
* Pedestrian Detector v0.1    2013-08
*
* Fabio Reis
* [freis-at-isr.ist.utl.pt]
*
* Please email me if you find bugs, or have suggestions or questions!
* Licensed under the Simplified BSD License [see external/bsd.txt]
*******************************************************************************/

#include "../include/pedDetector.hpp"

/*/ Defining our parser error handler (not defined by library)
void rapidxml::parse_error_handler(const char *what, void *where){
  std::cout << "Parse error: " << what << "\n";
  std::abort();
}
//*/

//*/
helperXMLParser::helperXMLParser(string filename){
  // Read the source file
  ifstream in(filename.c_str());

  // Prepare it for RapidXML parser
  std::vector<char> buffer((std::istreambuf_iterator<char>(in)), 
                            std::istreambuf_iterator<char>( ));
  buffer.push_back('\0');
  
  // Rapidxml document class and parser
  rapidxml::xml_document<> doc;
  doc.parse<0>(&buffer[0]);

  // Find our root node
	rapidxml::xml_node<> *root_node = doc.first_node("detector");
	
	// Verbose node
	rapidxml::xml_node<> *verboseN = root_node->first_node("verbose");
	verbose = (atoll(verboseN->first_attribute("value")->value()) != 0);
	
	// Pyramid node
	rapidxml::xml_node<> *pyramidN = root_node->first_node("pyramid");
	nrChannels = atoll(pyramidN->first_attribute("nrChannels")->value());
  nrScales = atoll(pyramidN->first_attribute("nrScales")->value());
  minH = atoll(pyramidN->first_attribute("minH")->value());
  minW = atoll(pyramidN->first_attribute("minW")->value());
  
  // Classifier node
	rapidxml::xml_node<> *classifierN = root_node->first_node("classifier");
	widthOverHeight = atof(classifierN->first_attribute("widthOverHeight")->value());
	shrinkFactor = atoll(classifierN->first_attribute("shrinkFactor")->value());
	
	theoWWidth = atoll(classifierN->first_attribute("theoWWidth")->value());
	theoWHeight = atoll(classifierN->first_attribute("theoWHeight")->value());
	
	theoActWWidth = atof(classifierN->first_attribute("theoActWWidth")->value());
	theoActWHeight = atof(classifierN->first_attribute("theoActWHeight")->value());
	
	rectFile = classifierN->first_attribute("rectFile")->value();
	nrFeatures = atoll(classifierN->first_attribute("nrFeatures")->value());
	nrProp = atoll(classifierN->first_attribute("nrProp")->value());
	
	classFile = classifierN->first_attribute("classFile")->value();
	nrClass = atoll(classifierN->first_attribute("nrClass")->value());
	nrCol = atoll(classifierN->first_attribute("nrCol")->value());
	
	nBaseFeatures = atoll(classifierN->first_attribute("nBaseFeatures")->value());
	nExtraFeatures = atoll(classifierN->first_attribute("nExtraFeatures")->value());
}

helperXMLParser::~helperXMLParser(){

}

void helperXMLParser::print(){
  cout << "Verbose            : " << verbose          << endl
       << "Nr. channels       : " << nrChannels       << endl
       << "Nr. scales         : " << nrScales         << endl
       << "Minimum height     : " << minH             << endl
       << "Minimum width      : " << minW             << endl
       << "widthOverHeight    : " << widthOverHeight  << endl
       << "Shrink factor      : " << shrinkFactor     << endl
       << "Theo. width        : " << theoWWidth       << endl
       << "Theo. height       : " << theoWHeight      << endl
       << "Theo. Act. width   : " << theoActWWidth    << endl
       << "Theo. Act. height  : " << theoActWHeight   << endl
       << "Rect. file         : " << rectFile         << endl
       << "Nr. features       : " << nrFeatures       << endl
       << "Nr. properties     : " << nrProp           << endl
       << "Class. file        : " << classFile        << endl
       << "Nr. class.         : " << nrClass          << endl
       << "Nr. cols           : " << nrCol            << endl
       << "nBaseFeatures      : " << nBaseFeatures    << endl
       << "nExtraFeatures     : " << nExtraFeatures   << endl ;
}
//*/

vector<Rect_<int> >* pedDetector(Mat img_original){
  // Lets open the configuration file
  helperXMLParser parsed = helperXMLParser("configuration.xml");
  if(parsed.verbose)
    parsed.print();
  
  // These are helper variables
  Mat image, imagef, imageO = img_original;
  
  /*
   * Converts Image to RGB
   * TODO :: This must be remade as image may\may not be in BGR
   */
  
  cvtColor(imageO, image, CV_BGR2RGB);
  image.convertTo(imagef, CV_32FC3, 1/255.0, 0);

  /*
   * Initialize image properties (misalign - controls memory mis-alignment)
   */
  const int h=imagef.size[0], w=imagef.size[1], misalign=1;
  int c = image.channels();
  
  /*
   * Converts Mat image to float*
   */
  float *img;
  img = convertFromMat(imagef, h, w, c, misalign);

  /*
   * Prepares Pyramid Input
   */
  int *sz = new int[3]{ h, w, c };
  pyrInput *pInput = new pyrInput();

  // Image Size
  delete(pInput->sz);
  pInput->sz = sz;

  // Number of Scale Approximations to compute between real scales
  pInput->nApprox = 0;

  // Minimum Dimensions
  delete(pInput->minDs);
  int *minDs = new int[2]{parsed.minH,parsed.minW};
  pInput->minDs = minDs;

  /*
   * Prepares the Strong Classifier Inputs
   */
  // Rectangles Data
  ClassRectangles *rectangles = new ClassRectangles(parsed.rectFile,
                                                    parsed.nrFeatures, 
                                                    parsed.nrProp);
  
  // Classifier Data
  ClassData *classData = new ClassData(parsed.classFile,
                                       parsed.nrClass, 
                                       parsed.nrCol);
  
  // Setup the classifier
  classifierInput *sctInput = new classifierInput(classData, 
                                                  rectangles,
                                                  parsed.verbose,
                                                  parsed.widthOverHeight,
                                                  parsed.shrinkFactor,
                                                  parsed.theoWWidth,
                                                  parsed.theoWHeight,
                                                  parsed.theoActWWidth,
                                                  parsed.theoActWHeight,
                                                  parsed.nBaseFeatures,
                                                  parsed.nExtraFeatures
                                                 );

  // Padding
  delete(pInput->pad);
  int *pad = new int[2]{sctInput->theoreticalVerticalPadding, sctInput->theoreticalHorizontalPadding};
  pInput->pad = pad;

  /*
   * Calculate Pyramids
   */
  pyrOutput *pOutput = chnsPyramid(img, pInput);
  
  /*
   * Running the detector
   */
  vector<cv::Rect_<int> >* rects = sctRun(pOutput, sctInput);

  if(parsed.verbose)
    cout << "Ended my work." << endl;
  
  /*
   * Free Memory
   */
  delete(minDs);
  delete(pInput);
  delete(pOutput);
  delete(rectangles);
  delete(classData);
  delete(sctInput);
  wrFree(img - misalign);
  return rects;
}
