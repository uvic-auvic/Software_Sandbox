#include <ros/ros.h>
#include "msg/newmsg.h"


int main(int argc, char **argv) {
    ros::init(argc, argv, "talker");
    ros::NodeHandle nh;

    ros::Publisher pub = nh.advertise<msg::newmsg>("chatter",5000);
    ros::Rate loop_rate(10);

    int count = 0;
    while(ros::ok()) {
        msg::newmsg newmsg;
        newmsg.number = count++;
        newmsg.astring = "Hello world";
        
        pub.publish(newmsg);
        ros::spinOnce();
        loop_rate.sleep();
    }

    // We never get here unless roscore quits
    return 0;   
}


 


