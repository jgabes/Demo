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
	//ifstream  iFile(blah, ios::binary);
	deserialize(blah)>>my_sp;

	//deserialize(my_sp, iFile);


	ROS_INFO_STREAM("Waiting for cameras to come online.");
	ros::Duration(3).sleep();

	//int num_cameras = initialize_devices();
	//if (num_cameras != 2)
	//	cerr << "WARNING: FaceNode expecting 2 camera configuration." << endl;

	ROS_INFO_STREAM("FaceTracking Node - Waiting for cameras to warm boot");
	ros::Duration(1).sleep();

	//int rc = pthread_create(&device_thread_, NULL, devicePoll, NULL);
	//if (rc) {
	//	cerr << "ERROR; return code from pthread_create() is " << rc << endl;
	//}

	ros::Rate loop_rate(30); //set goal refresh rate (hz)


	boost::shared_ptr<rsROSDevice> my_cam;
	//rsROSDevice* my_cam = new rsROSDevice;


	//delete my_cam;
	while (ros::ok())
	{

		cv::imshow("test", my_cam->get_frame());
		cv::waitKey(2);
		ros::spinOnce();//Spin me right round
		loop_rate.sleep(); //Sleep to hit goal refresh rate
	}



	//////////////////////////////////////////////////////////
	// DONE
	//////////////////////////////////////////////////////////
	runApp = false;

	int idx = 0;

	for (auto str : depth_frame_id)
		delete &str;

	ROS_INFO_STREAM("Eyes Closed");
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
	}
	//dlib::proxy_deserialize pds(blah) >> sp;
	//my_sp.deserialize(my_sp,iFile);

	// Service

}



//void hand_track(cv::Mat& image)
//{
//
//	MyImage m(0);
//		HandGesture hg;
//		init(&m);
//		m.src=image;
//
//
//	    namedWindow("img1",CV_WINDOW_KEEPRATIO);
//		out.open("out.avi", CV_FOURCC('M', 'J', 'P', 'G'), 15, m.src.size(), true);
//		waitForPalmCover(&m);
//		average(&m);
//		destroyWindow("img1");
//		initWindows(m);
//		initTrackbars();
//		for(;;){
//			hg.frameNumber++;
//			m.cap >> m.src;
//			flip(m.src,m.src,1);
//			pyrDown(m.src,m.srcLR);
//			blur(m.srcLR,m.srcLR,Size(3,3));
//			cvtColor(m.srcLR,m.srcLR,ORIGCOL2COL);
//			produceBinaries(&m);
//			cvtColor(m.srcLR,m.srcLR,COL2ORIGCOL);
//			makeContours(&m, &hg);
//			hg.getFingerNumber(&m);
//			showWindows(m);
//			out << m.src;
//			//imwrite("./images/final_result.jpg",m.src);
//	    	if(cv::waitKey(30) == char('q')) break;
//		}
//		destroyAllWindows();
//		out.release();
//		m.cap.release();
//	    return 0;
//
//}

