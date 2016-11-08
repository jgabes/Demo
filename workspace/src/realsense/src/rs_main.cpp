//
//      ()_()        Walt Disney Imagineering       ()_()
//       (_)      Research and Development, Inc      (_)
//
//
//! \file  src/realsense_face_node/src/realsense_face_node_spoof.hh
//! A ROS node for particle filter tracking of faces
//
//====================================================================

#include "rs_main.hh"
float DEPTH_THRESHOLD = 2.5;
int counter = 0;
bool state_change_active = false;
int state = 0;
bool finding_hand = true;
MyImage m(0);
HandGesture hg;


//dlib::image_window win;

//
// Real sense boiler plate error processing.
//
void checkError() {
	if (rs_error_) {
		ROS_ERROR_STREAM(
				"RealSense Camera - Error calling " << rs_get_failed_function(rs_error_) << " ( " << rs_get_failed_args(rs_error_) << " ): \n" << rs_get_error_message(rs_error_) << " \n");
		rs_free_error(rs_error_);

		ros::shutdown();

		exit(EXIT_FAILURE);
	}
}

short sharpen(cv::Mat &inputframe) {
	cv::Mat tmpMat;

	// aperture size of 1 corresponds to the correct matrix
	cv::Laplacian(inputframe, tmpMat, CV_16S, 3, 1, 0, cv::BORDER_DEFAULT);

	short maxLap = -32767;
	short *imgData = (short *) tmpMat.data;
	for (int i = 0; i < (color_width_ * color_height_) / 2; i++) {
		if (imgData[i] > maxLap)
			maxLap = imgData[i];
	}

	cv::Mat abs_dst;
	cv::convertScaleAbs(tmpMat, abs_dst);
	inputframe += abs_dst;

	return maxLap;
}

cv::Mat get_depth_image(int theIdx, rs_device *rs_device_,
		cv::Mat &image_color, const uint16_t *theDepthImg) {

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

#define USE_DESKTOP 1

//void *devicePoll(void *args) {
//}
std::vector<cv::Mat> devicePoll(){
	//void *devicePoll(void *args) {
	// void devicePoll(){

	const uint8_t *color_image[devices.size()];
	const uint16_t *depth_image[devices.size()];
	const uint16_t *aligned_depth_image[devices.size()];

	//	while (ros::ok()) {
	//std::cout << "frame" << std::endl;
	int idx = 0;  // reset
	cv::Mat rgb_img = cv::Mat(color_height_, color_width_, CV_8UC3,
			cv::Scalar(0, 0, 0));
	cv::Mat rgb_img_out = rgb_img;

	cv::Mat depth_img = cv::Mat(color_height_, color_width_, CV_32FC1,
			cv::Scalar(0, 0, 0));



#ifdef USE_DESKTOP
	rs_device * rs_device_ = devices[0];
#else
	for (auto rs_device_ : devices)
#endif
	{
		if (rs_is_device_streaming(rs_device_, 0) == 1) {
			rs_wait_for_frames(rs_device_, &rs_error_);
			checkError();
			time_stamp_ = ros::Time::now();

			color_image[idx] = (const uint8_t *) rs_get_frame_data(
					rs_device_, RS_STREAM_COLOR, &rs_error_);
			depth_image[idx] = (const uint16_t *) rs_get_frame_data(
					rs_device_, RS_STREAM_DEPTH_ALIGNED_TO_COLOR, &rs_error_);

			checkError();



			rgb_img.data = (unsigned char *) color_image[idx];
			cv::cvtColor(rgb_img, rgb_img, cv::COLOR_RGB2BGR);


			depth_img=get_depth_image(idx, rs_device_, rgb_img,
					depth_image[idx]);

			/*
				cv::cvtColor(rgb_img, rgb_img_out, cv::COLOR_BGR2RGB);
				std::string also_Result = "/home/shadylady/Pictures/new_image_"
								+ std::to_string(counter) + ".jpg";
				cv::String new_filename(also_Result);

				cv::imwrite(new_filename, rgb_img);
				counter++;
			 */



		}

		idx++;
	}

	//	}
	std::vector<cv::Mat> out_vec = {rgb_img, depth_img};
	return out_vec;
}

int initialize_devices() {

	// ROS_ERROR_STREAM("front 1 camera idx:" << front1_camera);

	// std::cout << "front 1 camera idx: " << front1_camera <<  std::endl;

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
	if (num_of_cameras < 1) {
		ROS_ERROR_STREAM(
				"RealSense Camera - No cameras are connected. face node");
		return false;
	}

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
				devices.push_back(dev);

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
	for (auto dev : devices) {
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
	}

	// Capture 30 frames to give autoexposure, etc. a chance to settle
	// let one camera do this to block
	for (int i = 0; i < 10; ++i) {
		for (auto dev : devices)
			rs_wait_for_frames(dev, &rs_error_);
	}

	cout << "number of cameras being used: " << num_of_cameras << endl;

	return num_of_cameras;
}

//////////////////////////////////////////////////////////
// Service
//////////////////////////////////////////////////////////
int main(int argc, char **argv) {

	// Initialize ROS
	ros::init(argc, argv, "eyes");
	ROS_INFO_STREAM("Starting Eyes");
	ros::NodeHandle nh;

	pan_pub = nh.advertise<std_msgs::Int16>("servo_pan", 1); //create a publisher
	tilt_pub = nh.advertise<std_msgs::Int16>("servo_tilt", 1); //create a publisher

	//////////////////////////////////////////////////////////
	// Output RGB
	//////////////////////////////////////////////////////////
	std::string blah("/home/shadylady/Demo/workspace/src/realsense/src/shape_predictor_68_face_landmarks.dat");
	deserialize(blah)>>my_sp;

	ROS_INFO_STREAM("Waiting for cameras to come online.");
	ros::Duration(3).sleep();

	int num_cameras = initialize_devices();

	ROS_INFO_STREAM("FaceTracking Node - Waiting for cameras to warm boot");
	ros::Duration(1).sleep();

	ros::Rate loop_rate(60); //set goal refresh rate (hz)

	init(&m);

	while (ros::ok())
	{
		int hand_count = 0;
		std::vector<cv::Mat> frames = devicePoll();
		switch(state){
		case 0:
		{
			track_faces(frames);
			break;
		}
		case 1:
		{
			cv::imshow("state changer",frames[0]);
			cv::waitKey(2);
			m.src=frames[0];
			if (hand_count < 50)
			{
				get_palm(hand_count);
			}


			break;
		}
		case 2:
		{
			break;
		}
		}


		ros::spinOnce();//Spin me right round

		loop_rate.sleep(); //Sleep to hit goal refresh rate
	}


	//	ros::spin();
	//////////////////////////////////////////////////////////
	// DONE
	//////////////////////////////////////////////////////////
	runApp = false;

	int idx = 0;

	for (auto str : depth_frame_id)
		delete &str;

	ROS_INFO_STREAM("Eyes Closed");
}

void track_faces(std::vector<cv::Mat>& frames)
{
	cv::Mat rgb_img = frames[0];
	cv::Mat depth_img = frames[1];

	std::vector<dlib::rectangle> faces = detect_faces(rgb_img);
	long lgArea = 0;
	dlib::rectangle large_face;
	for (auto face : faces) {

		cv::Point tr(face.right(), face.top());
		cv::Point bl(face.left(), face.bottom());
		cv::rectangle(rgb_img, tr, bl, cv::Scalar(255,0,0), 3);


		if (face.area() > lgArea)
		{
			lgArea = face.area();
			large_face = face;
		}
	}
	std_msgs::Int16 face_height_error;
	std_msgs::Int16 face_width_error;
	if (lgArea >0)
	{
		cv::Point tr(large_face.right(), large_face.top());
		cv::Point bl(large_face.left(), large_face.bottom());
		cv::rectangle(rgb_img, tr, bl, cv::Scalar(0,255,0), 6);
		int face_center_height = (large_face.top()+large_face.bottom())/2;
		int face_center_width = (large_face.left()+large_face.right())/2;


		face_height_error.data = face_center_height-CENTER_HEIGHT;
		face_width_error.data = face_center_width-CENTER_WIDTH;
		//std::cout<<"height error: "<<face_height_error.data<<std::endl;
		//std::cout<<"width error: "<<face_width_error.data<<std::endl;

		if (abs(face_height_error.data) < 50){face_height_error.data = 0;}
		if (abs(face_width_error.data) < 50){face_width_error.data = 0;}

		facial_landmarks(large_face, rgb_img);
		state_change_active = true;
	}
	else
	{
		face_width_error.data = 0;
		face_height_error.data = 0;
	}


	pan_pub.publish(face_width_error);
	tilt_pub.publish(face_height_error);

	cv::imshow("rgb", rgb_img);
	cv::imshow("depth", depth_img);
	cv::waitKey(2);
	if (state_change_active){
		state_change(depth_img);
	}
}

std::vector<dlib::rectangle> detect_faces(cv::Mat rgb_img)
																		{
	cv::Mat mGrey;
	// Grayscale Image of eyes camera
	mGrey = cv::Mat(color_height_, color_width_, CV_8UC3,
			cv::Scalar(0, 0, 0));


	// Convert RGB Images to Greyscale images
	cv::cvtColor(rgb_img, mGrey, CV_BGR2GRAY);


	dlib::cv_image<unsigned char> dlibimg(mGrey);
	std::vector<dlib::rectangle> faces = detector(dlibimg);




	return faces;
																		}

void facial_landmarks(dlib::rectangle& face, cv::Mat& image){

	cv_image<bgr_pixel> dlibimg(image);
	full_object_detection shape = my_sp(dlibimg, face);
	for (int i=0; i<shape.num_parts(); i++)
	{
		dlib::point dlib_pt = shape.part(i);
		//dlib::vector<long,2> vec = dlib_pt;
		cv::Point cv_pt(dlib_pt(0), dlib_pt(1));
		cv::circle(image, cv_pt, 1, cv::Scalar(255,0,0), 1);
	}
	return;
}

void state_change(cv::Mat& image)
{

	cv::Mat mGrey = cv::Mat(color_height_, color_width_, CV_8UC3,
			cv::Scalar(0, 0, 0));
	cv::cvtColor(image, mGrey, CV_BGR2GRAY);
	if(cv::countNonZero(mGrey)<20)
	{
		cout<<"STATE CHANGE"<<std::endl;
		state_change_active = true;
		if (state!=1){state = 1;}
		else {state=1;}
	}
	//dlib::proxy_deserialize pds(blah) >> sp;
	//my_sp.deserialize(my_sp,iFile);

	// Service

}

void get_palm(int count)
{
	namedWindow("img1",CV_WINDOW_KEEPRATIO);
	out.open("out.avi", CV_FOURCC('M', 'J', 'P', 'G'), 15, m.src.size(), true);
	waitForPalmCover(&m, count);
	average(&m);
	destroyWindow("img1");
}

void hand_track(cv::Mat& image, int count)
{






	initWindows(m);
	initTrackbars();
	for(;;){
		hg.frameNumber++;
		m.cap >> m.src;
		flip(m.src,m.src,1);
		pyrDown(m.src,m.srcLR);
		blur(m.srcLR,m.srcLR,Size(3,3));
		cvtColor(m.srcLR,m.srcLR,ORIGCOL2COL);
		produceBinaries(&m);
		cvtColor(m.srcLR,m.srcLR,COL2ORIGCOL);
		makeContours(&m, &hg);
		hg.getFingerNumber(&m);
		showWindows(m);
		out << m.src;
		//imwrite("./images/final_result.jpg",m.src);
		if(cv::waitKey(30) == char('q')) break;
	}
	destroyAllWindows();
	out.release();
	m.cap.release();
	return 0;

}

