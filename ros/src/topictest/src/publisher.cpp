#include <sstream>
#include <ros/ros.h>
#include "std_msgs/String.h"


int main(int argc, char **argv) {
    ros::init(argc, argv, "talker");
    ros::NodeHandle nh;

    // Set up the publisher which is the object which sends our messages
    ros::Publisher chatter_pub = nh.advertise<std_msgs::String>("chatter",5000);

    // The rate at which we 'spin', i.e how often we send data out
    ros::Rate loop_rate(10);

    int count = 0;
    while(ros::ok()) {
        // ROS uses their own String implementation, which is what we need to send data through topics
        std_msgs::String msg;

        std::stringstream ss;
        ss << "hello world " << count++;
        msg.data = ss.str();
        
        // Send out the data
        chatter_pub.publish(msg);

        // Let ROS do its thing
        ros::spinOnce();

        // sleep so we dont overrun our loop rate
        loop_rate.sleep();
    }

    // We never get here unless roscore quits
    return 0;   
}


 


