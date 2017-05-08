#include "ros/ros.h"
#include "std_msgs/String.h"

void chatterCallback(const std_msgs::String::ConstPtr& msg) {
    ROS_INFO("Message: %s", msg->data.c_str());
}

int main(int argc, char **argv){
    // setup the rosnode
    ros::init(argc, argv, "listener");

    // Add the node handle, so we can handle ros things
    ros::NodeHandle n;

    // setup a subscriber. the "chatter" is the topic name
    // The 1000 is the buffer size, so we can take 1000 messages before handling them
    // chatterCallback is the function that does something with the topic message
    ros::Subscriber sub = n.subscribe("chatter", 1000, chatterCallback);

    // This is basically the ros while-loop. Your callback can only be called if spinning
    ros::spin();
    return 0;
}
