/*******************************************************************************
* Pedestrian Detector v0.1    2013-08
*
* Fabio Reis
* [freis-at-isr.ist.utl.pt]
*
* Please email me if you find bugs, or have suggestions or questions!
* Licensed under the Simplified BSD License [see external/bsd.txt]
*******************************************************************************/

/*
 * General includes
 */
#include <iostream>
#include <string>
#include <sstream>

/*
 * ROS specific includes
 */
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

/*
 * Our detector
 */
#include "pedDetector.hpp"

namespace cvb = cv_bridge;
namespace enc = sensor_msgs::image_encodings;

/*
 * Helper class that wraps the code for ROS
 */
class PedDetector {

private:
// ROS Handling
  ros::NodeHandle n_;
  ros::NodeHandle n_priv;
  image_transport::ImageTransport it_;

// Parameters set during initialization.
  std::string _inputVideoPortName;
  image_transport::Subscriber  _inputVideoPort;

  std::string _outputVideoPortName;
  image_transport::Publisher _outputVideoPort;

  std::string _outputDataPortName;
  ros::Publisher _outputDataPort;

// cv_bridge Pointer for the ROS original image, we will change this image
  cv_bridge::CvImagePtr ros_cv_ptr;

// should we output the image with the bounding boxes
  bool outputImage;

public:

  PedDetector(); //constructor
  PedDetector(ros::NodeHandle); //constructor
  ~PedDetector(); //destructor

// Callback that calls the detector code given a new image
  void detectorCallback(const sensor_msgs::ImageConstPtr& msg_ptr);
  
// Setup functions
  bool open(); //member to set the object up.
  virtual bool close(); //member to close the object.
  
// Bounding box results. OpenCV Rect_<int> structure
  vector<cv::Rect_<int> >* results;
};


