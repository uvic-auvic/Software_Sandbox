#include <ros/ros.h>
#include "servicestest/myservice.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "addfiveclient");
    ros::NodeHandle nh("~");

    ros::ServiceClient c1 = nh.serviceClient<servicestest::myservice>("/addfiveserver/add_success");
    servicestest::myservice srv1;
    srv1.request.number = 10;

    if (c1.call(srv1)) {
      ROS_INFO("Response: %d", srv1.response.number_plus_5);
    } else {
      ROS_ERROR("Failed to call service add_success");
      return 1;
    }

    ros::ServiceClient c2 = nh.serviceClient<servicestest::myservice>("/addfiveserver/add_fail");
    servicestest::myservice srv2;
    srv2.request.number = 10;

    if (c2.call(srv2)) {
      ROS_INFO("Response: %d", srv2.response.number_plus_5);
    } else {
      ROS_ERROR("Failed to call service add_fail.");
      return 1;
    }

    return 0;
}
