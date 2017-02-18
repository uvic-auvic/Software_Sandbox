#include <stdio.h>

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "serial/serial.h"

void chatterCallback(const std_msgs::String::ConstPtr& msg)
{

  //std::string delimiter = "\n";
  //std::string token = msg.substr(0, msg.find(delimiter)); // token is "scott
    std::string a = "0123456789abcdefghij";
    std::string sub1 = a.substr(2);
       std::cout << sub1 << '\n';
    ROS_INFO("I heard: [%s]", msg->data.c_str());

}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "listener");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("chatter", 1000, chatterCallback);
  ros::spin();
  return 0;
}
