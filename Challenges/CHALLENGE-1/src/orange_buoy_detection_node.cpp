#include "ros/ros.h"
#include "image_transport/image_transport.h"
#include "opencv2/opencv.hpp"
#include "cv_bridge/cv_bridge.h"
#include <stdio.h>
#include <iostream>
#include "std_msgs/String.h"

using namespace cv;

void imageCallback(const sensor_msgs::ImageConstPtr& msg){

	cv_bridge::CvImagePtr cv_ptr;
	try{
		cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
	}
	catch (cv_bridge::Exception e){
		ROS_ERROR("couldn't convert from '%s' to 'bgr8'.", msg->encoding.c_str());
	}

	Mat frame = cv_ptr->image; 

  imshow( "view", frame );
		cv::waitKey(30);

}

int main(int argc, char** argv){
	ros::init(argc, argv, "orange_buoy_detection_node");
	ros::NodeHandle nh;

  image_transport::ImageTransport it(nh);
	image_transport::Subscriber sub = it.subscribe("camera", 1, imageCallback);
	ros::spin();
	cv::destroyWindow("view");
	return 0;
	
}