#include "ros/ros.h"
#include "grabber/AddTwoInts.h"
#include "grabber/GrabberCommand.h"
#include <cstdlib>

/**
 * Example of how to send a command to the grabber's service_node
 * (could also be used to control the grabber from a terminal)
 */

int main(int argc, char **argv)
{
  ros::init(argc, argv, "client_node");
  if (argc != 3)
  {
    ROS_INFO("usage: client_node X Y");
    return 1;
  }

  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<grabber::GrabberCommand>("grabber_command");
  grabber::GrabberCommand srv;
  srv.request.command = argv[1];
  srv.request.param = atoll(argv[2]);
  if (client.call(srv))
  {
      ROS_INFO("GOOD");
      if(!srv.response.success)
      {
          ROS_WARN("Unrecognized command and/or parameters");
      }
  }
  else
  {
      ROS_INFO("BAD");
      return 1;
  }

  return 0;
}
