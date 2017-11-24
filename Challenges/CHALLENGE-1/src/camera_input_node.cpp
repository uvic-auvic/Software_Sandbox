#include <opencv2/opencv.hpp>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <ros/ros.h>
#include <string>

using namespace cv;

int main(int argc, char ** argv){
  ros::init(argc, argv, "camera_input_node");

  ros::NodeHandle nh;
  Mat img;
  //load a picture or a video, comment out respective section
  //while loading other type
  VideoCapture cap("test-footage.mp4");
  
  //img = imread(argv[1], 1);
  //if( !img.data ){
  //  return -1;
  //}
  

  image_transport::ImageTransport it(nh);
  image_transport::Publisher publisher = it.advertise("camera", 1);

  while(nh.ok()){
      //video line below
      cap >> img;

      if(img.empty()) break;

      //test line to see input 
      imshow("test", img);
      sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", img).toImageMsg();

      publisher.publish(msg);

      if( waitKey(10) == 27 ) break;

      ros::spinOnce();
      
  }

  return 0;
}