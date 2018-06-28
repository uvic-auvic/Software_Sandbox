#include <ros/ros.h>
#include "msg/newmsg.h"


class reciever {
public:
    reciever() {}
    void recieve(const msg::newmsg::ConstPtr &mymsg)
    {
        ROS_INFO("[%d] %s", mymsg->number, mymsg->astring.c_str());
    }

};

int main(int argc, char **argv){
    // setup the rosnode
    ros::init(argc, argv, "listener");

    ros::NodeHandle nh;
    reciever r;
    ros::Subscriber sub 
        = nh.subscribe<msg::newmsg>("chatter", 1000, &reciever::recieve, &r);
    ros::spin();
    return 0;
}
