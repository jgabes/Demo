/*
 * rsROSDevice.h
 *
 *  Created on: Nov 7, 2016
 *      Author: shadylady
 */

#ifndef RSROSDEVICE_H_
#define RSROSDEVICE_H_

#include <ros/ros.h>
#include <stdio.h>

#include <librealsense/rs.hpp>
#include <librealsense/rscore.hpp>

#include <opencv2/core/core.hpp>
#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <string>


#include <iostream>
#include <algorithm>
#include <vector>
#include <math.h>
//#include <string>

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/Int16.h>


#include <librealsense/rs.hpp>
#include <librealsense/rscore.hpp>

#include <opencv2/core/core.hpp>
#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <stdio.h>
#include <iostream>
#include <iomanip>
#include <locale>
#include <sstream>


#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/imgproc/types_c.h"
#include "opencv2/highgui/highgui_c.h"
#include <opencv2/opencv.hpp>
#include <stdlib.h>
#include <iostream>
#include "myImage.hpp"
#include "roi.hpp"
#include "handGesture.hpp"
#include <cmath>
#include "fake_main.hpp"
#include <boost/shared_ptr.hpp>
#include "rsROSDevice.h"


using namespace std;






class rsROSDevice {
public:
	rsROSDevice();
	cv::Mat get_frame();
	virtual ~rsROSDevice();
	cv::Mat getColorFrame();
	cv::Mat getDepthFrame();

private:
	cv::Mat get_depth_image(int theIdx, rs_device *rs_device_,
			cv::Mat &image_color, const uint16_t *theDepthImg);



	static std::vector<rs_device *> devices;
	rs_device* device;
	const uint8_t *color_image;
	const uint16_t *depth_image;
	const uint16_t *aligned_depth_image;
	cv::Mat color_frame;
	cv::Mat depth_frame;


	void checkError();


	const int NUMBER_OF_FACE_CAMERAS = 2;
	const int COLOR_HEIGHT = 480;
	const int COLOR_WIDTH = 640;
	const int DEPTH_HEIGHT = 480;
	const int DEPTH_WIDTH = 640;
	float DEPTH_THRESHOLD = 2.5;
	const int MAX_Z = 6;  // in meters

	rs_error *rs_error_ = 0;
	rs_intrinsics color_intrinsic;
	rs_extrinsics z_extrinsic;
	rs_intrinsics z_intrinsic;

};



#endif /* RSROSDEVICE_H_ */
