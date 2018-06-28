#include <ros/ros.h>
#include "servicestest/myservice.h"

bool add_success(servicestest::myservice::Request  &req,
         servicestest::myservice::Response &res)
{
  res.number_plus_5 = req.number + 5;
  return true;
}

bool add_fail(servicestest::myservice::Request  &req,
         servicestest::myservice::Response &res)
{
  res.number_plus_5 = req.number + 5;
  return false;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "addfiveserver");
  ros::NodeHandle nh("~");

  ros::ServiceServer s1 = nh.advertiseService("add_success", add_success);
  ros::ServiceServer s2 = nh.advertiseService("add_fail", add_fail);
  ROS_INFO("Ready to add 5 to number.");
  ros::spin();

  return 0;
}