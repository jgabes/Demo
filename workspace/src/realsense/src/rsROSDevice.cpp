/*
 * rsROSDevice.cpp
 *
 *  Created on: Nov 7, 2016
 *      Author: shadylady
 */

#include "rsROSDevice.h"

rsROSDevice::rsROSDevice() {
	// TODO Auto-generated constructor stub


	// ROS_ERROR_STREAM("front 1 camera idx:" << front1_camera);

	// std::cout << "front 1 camera idx: " << front1_camera <<  std::endl;

	std::string front1_camera;
	std::string front2_camera;
	rs_context *rs_context_;
	static std::vector<std::string> color_frame_id;
	static std::vector<std::string> depth_frame_id;

	int idx = 0;

	// Convert to char array
	const char *front1_camera_char = front1_camera.c_str();
	const char *front2_camera_char = front2_camera.c_str();

	//	std::cout << "front 1 camera idx: " << front1_camera <<  std::endl;

	std::vector<const char *> inventory = {
			"2441012451"
	};

	rs::log_to_console(rs::log_severity::warn);

	rs_context_ = rs_create_context(RS_API_VERSION, &rs_error_);

	std::vector<rs_device *> tmpDevList;

	int num_of_cameras = rs_get_device_count(rs_context_, NULL);


	for (int i = 0; i < num_of_cameras; ++i) {
		std::cout << "query device idx: " << i << "::" << num_of_cameras
				<< std::endl;
		tmpDevList.push_back(rs_get_device(rs_context_, i, &rs_error_));
	}

	for (auto devID : inventory) {
		for (auto dev : tmpDevList) {
			const char *serialId = rs_get_device_serial(dev, &rs_error_);
			std::cout << " DevID - " << devID << "   Serial ID - " << serialId
					<< std::endl;
			if (strcmp(devID, serialId) == 0) {
				std::cout << "Adding device IDX " << serialId << "(" << idx
						<< ")" << std::endl;
				device = dev;
				//devices.push_back(dev);

				// create TFs
				std::ostringstream linkSS;
				linkSS << "_link_" << idx;
				string dsFrameStr = *(new string("facedepth"));
				dsFrameStr += linkSS.str();
				string csFrameStr = *(new string("facecamera"));
				csFrameStr += linkSS.str();
				color_frame_id.push_back(csFrameStr);
				depth_frame_id.push_back(dsFrameStr);

			} else
				std::cout << "SKIPPING device " << serialId << "(" << idx << ")"
				<< std::endl;
		}
	}

	// Configure and start our devices
	idx = 0;
	//for (auto dev : devices) {
	rs_device* dev = device;
		// rs_set_device_option(dev,RS_OPTION_COLOR_ENABLE_AUTO_EXPOSURE,1,
		// &rs_error_);
		// checkError();
		rs_set_device_option(dev, RS_OPTION_COLOR_BACKLIGHT_COMPENSATION, 0,
				&rs_error_);
		rs_set_device_option(dev, RS_OPTION_COLOR_ENABLE_AUTO_WHITE_BALANCE, 0,
				&rs_error_);

		rs_set_device_option(dev, RS_OPTION_COLOR_BRIGHTNESS, 22, &rs_error_);
		rs_set_device_option(dev, RS_OPTION_COLOR_CONTRAST, 35, &rs_error_);
		rs_set_device_option(dev, RS_OPTION_COLOR_GAIN, 0, &rs_error_);
		rs_set_device_option(dev, RS_OPTION_COLOR_SHARPNESS, 1, &rs_error_);
		rs_set_device_option(dev, RS_OPTION_COLOR_WHITE_BALANCE, 8000,
				&rs_error_);
		checkError();
		// rs_enable_stream_preset (dev, RS_STREAM_COLOR, RS_PRESET_BEST_QUALITY,
		// &rs_error_);
		cout << "RGB W: " << COLOR_WIDTH << " H: " << COLOR_HEIGHT << endl;
		rs_enable_stream(dev, RS_STREAM_COLOR, COLOR_WIDTH, COLOR_HEIGHT,
				RS_FORMAT_RGB8, 30, &rs_error_);
		checkError();
		rs_enable_stream(dev, RS_STREAM_DEPTH, DEPTH_WIDTH, DEPTH_HEIGHT,
				RS_FORMAT_Z16, 30, &rs_error_);
		checkError();

		//	camera_info_[idx] = new sensor_msgs::CameraInfo();
		//	camera_info_ptr_[idx] = sensor_msgs::CameraInfoPtr(camera_info_[idx]);

		rs_intrinsics intrinsic;
		rs_get_stream_intrinsics(dev, RS_STREAM_COLOR, &intrinsic, &rs_error_);
		checkError();


		rs_start_device(dev, &rs_error_);
		checkError();

		idx++;

		std::cout << "done." << std::endl;
	//}

	// Capture 30 frames to give autoexposure, etc. a chance to settle
	// let one camera do this to block
	for (int i = 0; i < 10; ++i) {
		//for (auto dev : devices)

			rs_wait_for_frames(device, &rs_error_);
	}

	cout << "number of cameras being used: " << num_of_cameras << endl;




}

cv::Mat rsROSDevice::rsROSDevice::get_frame() {
	int idx=0;
//	for (auto rs_device_ : devices)
	//{
	rs_device* rs_device_ = device;
		if (rs_is_device_streaming(rs_device_, 0) == 1) {
			rs_wait_for_frames(rs_device_, &rs_error_);
			checkError();

			color_image = (const uint8_t *) rs_get_frame_data(
					rs_device_, RS_STREAM_COLOR, &rs_error_);
			depth_image = (const uint16_t *) rs_get_frame_data(
					rs_device_, RS_STREAM_DEPTH_ALIGNED_TO_COLOR, &rs_error_);

			checkError();
			cv::Mat rgb_img = cv::Mat(COLOR_HEIGHT, COLOR_WIDTH, CV_8UC3,
					cv::Scalar(0, 0, 0));
			cv::Mat rgb_img_out = rgb_img;

			cv::Mat depth_img = cv::Mat(COLOR_HEIGHT, COLOR_WIDTH, CV_32FC1,
					cv::Scalar(0, 0, 0));



			rgb_img.data = (unsigned char *) color_image;
			cv::cvtColor(rgb_img, rgb_img, cv::COLOR_RGB2BGR);


			depth_img=get_depth_image(idx, rs_device_, rgb_img,
					depth_image);
		}
		idx++;
	}
//}

rsROSDevice::~rsROSDevice() {
	// TODO Auto-generated destructor stub
}

cv::Mat rsROSDevice::getColorFrame() {
	return color_frame;
}

cv::Mat rsROSDevice::getDepthFrame() {
	return depth_frame;
}

cv::Mat rsROSDevice::rsROSDevice::get_depth_image(int theIdx,
		rs_device* rs_device_, cv::Mat& image_color,
		const uint16_t* theDepthImg) {


	rs_get_stream_intrinsics(rs_device_, RS_STREAM_DEPTH, &z_intrinsic,
			&rs_error_);
	checkError();
	rs_get_stream_intrinsics(rs_device_, RS_STREAM_COLOR, &color_intrinsic,
			&rs_error_);
	checkError();
	rs_get_device_extrinsics(rs_device_, RS_STREAM_DEPTH, RS_STREAM_COLOR,
			&z_extrinsic, &rs_error_);
	checkError();

	float depth_point[3], color_point[3], color_pixel[2], scaled_depth;
	unsigned char *color_data = (unsigned char *) image_color.data;
	const float depth_scale = rs_get_device_depth_scale(rs_device_, &rs_error_);
	checkError();  // Default value is 0.001
	scaled_depth = ((float) *theDepthImg) * depth_scale;

	// Fill the PointCloud2 fields.
	//#pragma omp parallel for collapse(2)
	cv::Mat Depth_Cloud(z_intrinsic.height, z_intrinsic.width, CV_32FC3);
	//cv::Mat Depth_Cloud;
	cv::Mat I(z_intrinsic.height, z_intrinsic.width, CV_32FC3);
	cv::Mat clip_color(z_intrinsic.height, z_intrinsic.width, CV_8UC3);
	//cv::Mat clip_hand(z_intrinsic.height, z_intrinsic.width, CV_8UC3);


	int channels = I.channels();
	int nRows = I.rows;
	int nCols = I.cols * channels;

	int i, j;
	float r, g, b;
	float x, y, z;
	if (I.isContinuous()) {
		nCols *= nRows;
		nRows = 1;
	}
	for (int y = 0; y < z_intrinsic.height; y++) { //i
		float* p = I.ptr<float>(y);
		uchar* p_cc = clip_color.ptr<uchar>(y);
		//uchar* p_hc = clip_hand.ptr<uchar>(y);
		for (int x = 0; x < z_intrinsic.width; x++) { // j
			scaled_depth = ((float) *theDepthImg) * depth_scale;
			float depth_pixel[2] = { (float) x, (float) y };
			if (scaled_depth>0.01)
			{
				rs_deproject_pixel_to_point(depth_point, &z_intrinsic, depth_pixel,
						scaled_depth);
				p[channels * x + 0]=depth_point[0];//r
				p[channels * x + 1]=depth_point[1];//g
				p[channels * x + 2]=depth_point[2];//b

				if (scaled_depth<DEPTH_THRESHOLD && scaled_depth > 0.01)
				{
					p_cc[channels * x + 2] = image_color.ptr<uchar>(y)[channels * x + 2]; //x
					p_cc[channels * x + 1] = image_color.ptr<uchar>(y)[channels * x + 1]; //y
					p_cc[channels * x + 0] = image_color.ptr<uchar>(y)[channels * x + 0]; //z

				}else{}

			}else{
				p_cc[channels * x + 2] = 0;//r
				p_cc[channels * x + 1] = 0;//g
				p_cc[channels * x + 0] = 0; //b
				p[channels * x + 2] = 0; //z
				p[channels * x + 1] = 0; //y
				p[channels * x + 0] = 0; //x
			}
			theDepthImg++;

		}

	}

	//cv::imshow("Pointer Style", clip_color);
	//cv::imshow("depth image", I);
	//cv::waitKey(2);

	return I;
}

void rsROSDevice::checkError() {
	if (rs_error_) {
		ROS_ERROR_STREAM(
				"RealSense Camera - Error calling " << rs_get_failed_function(rs_error_) << " ( " << rs_get_failed_args(rs_error_) << " ): \n" << rs_get_error_message(rs_error_) << " \n");
		rs_free_error(rs_error_);

		ros::shutdown();

		exit(EXIT_FAILURE);
	}
}
