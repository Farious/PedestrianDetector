/*******************************************************************************
* Pedestrian Detector v0.1    2013-08
*
* Fabio Reis
* [freis-at-isr.ist.utl.pt]
*
* Please email me if you find bugs, or have suggestions or questions!
* Licensed under the Simplified BSD License [see external/bsd.txt]
*******************************************************************************/

/** 
\defgroup icub_pedDet pedDet
@ingroup icub_module   

A real-time pedestrian detector.

Copyright (C) 2009 RobotCub Consortium
 
Author: Matteo Taiana, <A HREF="http://users.isr.ist.utl.pt/~mtaiana/">homepage</A>.

CopyPolicy: Released under the terms of the GNU GPL v2.0.

\section intro_sec Description

<b> INSERT MODEL DESCRIPTION HERE. </b>

<b>For an explanation on how to configure the tracker and bottom up modules, how to connect them, how to run them through the application manager, etc., please have a look at <A HREF="http://mediawiki.isr.ist.utl.pt/wiki/3D_ball_tracker">this page</A>.</b>

<b>The algorithm is described in the paper: "", please cite it if you use this detector in your research.</b>


You can watch a video of the detector in action <A HREF="http://www.youtube.com/watch?v=Xp8qUhfMzhU">here</A>.

\section lib_sec Libraries 
- Dollár's Toolbox
 
\section parameters_sec Parameters
Configuration is done through an initialization file (See for instance $ICUB_ROOT/main/app/pf3dTracker/conf/pf3dTracker.ini) Here is an example, with comments: 
\code 

\endcode 

\section portsa_sec Ports Accessed
The tracker need to be connected to a port that streams images, at the very least, in order to work.
 
\section portsc_sec Ports Created 
- /pf3dTracker/video:i receives the image stream given which the ball has to be tracked.

- /pf3dTracker/video:o produces images in which the contour of the estimated ball is highlighted. When the tracker is confident that it's tracking a ball, it draws the contour in green, when it is not confident (it's looking for a ball, but does not yet have a good estimate), it draws the contour in yellow.

- /pf3dTracker/data:o produces a stream of data in the format: X, Y, Z [meters], likelihood, U, V [pixels], seeing_object. <br>
X, Y and Z are the estimated coordinates of the tracked ball in the eye reference frame (they can be transformed to the root reference frame by module \ref eye2RootFrameTransformer "eye2RootFrameTransformer". The likelihood value indicates how confident the tracker is that the object it's tracking is the right ball (the lower the likelihood, the lower the confidence, but beware that even a perfect match will result in a value pretty far from 1). U and V are the estimated coordinates of the centre of the ball in the image plane, U is horizontal and V vertical, the origin is on the top left corner of the image. Seeing_object is a flag, it is set 1 when the likelihood is higher than a threshold specified in the initialization file, it is set to 0 otherwise. When the tracker experiences 5 consecutive images with seeing_object==0, the estimate is reset. This prevents the tracker from getting stuck on an unlikely target.

- /pf3dTracker/particles:i receives hypotheses on 3D poses of a ball, normally produced by the \ref icub_pf3dBottomup "pf3dBottomup" detection module. If the tracker does not receive anything on this port, it behaves normally, i.e., it needs more time to find a ball and start tracking it, after initialization.

- /pf3dTracker/particles:o produces data for the plotter. it is usually not active for performance reasons.

- /pf3dTracker/attention:o produces data for the attention system, in terms of a peak of saliency.
 

 
\section in_files_sec Input Data Files

\section out_data_sec Output Data Files 

\section tested_os_sec Tested OS
Ubuntu Linux

\author Matteo Taiana, <A HREF="http://users.isr.ist.utl.pt/~mtaiana/">homepage</A>.
*/ 

#include "../include/pedDetTracker_ROS.hpp"

using namespace std;

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "pedestrianDetector"); // set up ROS
  ros::NodeHandle n;
  PedDetector detector(n); //instantiate the tracker.
  ros::spin(); // pass control on to ROS

  return 0;
}


