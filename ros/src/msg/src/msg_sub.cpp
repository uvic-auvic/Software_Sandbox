#include <ros/ros.h>
#include "msg/newmsg.h"

void chatterCallback(const msg::newmsg::ConstPtr& mymsg) {
    ROS_INFO("[%d] %s", mymsg->number, mymsg->astring.c_str());
}

int main(int argc, char **argv){
    // setup the rosnode
    ros::init(argc, argv, "listener");

    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe<msg::newmsg>("chatter", 1000, chatterCallback);
    ros::spin();
    return 0;
}
