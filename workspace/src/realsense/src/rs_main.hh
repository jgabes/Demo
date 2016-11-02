//
//      ()_()        Walt Disney Imagineering       ()_()
//       (_)      Research and Development, Inc      (_)
//
//
//! \file  src/realsense_face_node/src/realsense_face_node.hh
//! A ROS node for reading realsense cameras and streaming the data as ROS
// RGB images and depth images
//
//====================================================================


#ifndef REALSENSE_FACE_NODE_SRC_REALSENSE_FACE_NODE_HH_
#define REALSENSE_FACE_NODE_SRC_REALSENSE_FACE_NODE_HH_

#include <iostream>
#include <algorithm>
#include <vector>
#include <math.h>
#include <string>

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/UInt8.h>


#include <librealsense/rs.hpp>

#include <opencv2/core/core.hpp>
#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>


#include <dlib/string.h>
#include <dlib/image_processing/frontal_face_detector.h>
#include <dlib/opencv.h>
#include <dlib/gui_widgets.h>
#include <dlib/image_io.h>
#include <dlib/image_transforms.h>


//======================================================================='

using namespace std;

std::string front1_camera;
std::string front2_camera;

const int NUMBER_OF_FACE_CAMERAS = 2;
const int COLOR_HEIGHT = 480;
const int COLOR_WIDTH = 640;
const int DEPTH_HEIGHT = 480;
const int DEPTH_WIDTH = 640;
const int MAX_Z = 6;  // in meters

int color_height_ = COLOR_HEIGHT;
int color_width_ = COLOR_WIDTH;
rs_error *rs_error_ = 0;
rs_context *rs_context_;
rs_intrinsics color_intrinsic;
rs_extrinsics z_extrinsic;
rs_intrinsics z_intrinsic;

/////
static std::vector<rs_device *> devices;
static std::vector<std::string> color_frame_id;
static std::vector<std::string> depth_frame_id;

double prevXPos = 0.0;
double prevYPos = 0.0;

double FaceDetectionThreshold = 25;

dlib::frontal_face_detector detector = dlib::get_frontal_face_detector();


int freq_count = 0;
int image_count = 0;

/////
// ROS needs
ros::Time time_stamp_;


/////
// Service
bool runApp = true;
pthread_t device_thread_;
//========================================================================


//Boilerplate realsense function to sense errors in the cameras
void checkError();

//realsense function to sharpen incoming image. I am critical of this
short sharpen(cv::Mat &inputframe);


//function to get depth images, transform them into the color image frame
// and publish them as std_msgs::Image with 3 layers of floats
cv::Mat get_depth_image(int theIdx, rs_device *rs_device_, cv::Mat &image_color, const uint16_t *theDepthImg);


//Main callback function which streams data from cameras and publishes the RGB
//channels
void *devicePoll(void *args);

//initialize realsense cameras. This needs to be modified to change camera
//serial numbers
int initialize_devices();

//find faces in an image using dlib
std::vector<dlib::rectangle> detect_faces(cv::Mat rgb_img);



#endif /* REALSENSE_FACE_NODE_SRC_REALSENSE_FACE_NODE_HH_ */
