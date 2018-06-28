#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>
#include <string>

int main(int argc, char **argv) {
    
    // Same boilerplate ROS things as usual
    ros::init(argc, argv, "talker");

    /* 
       This is a bit different. Usually the node handle is taken with no parameters, which actually creates a node handle with global scope.
       Passing '~' as a parameter creates the nodehandle with a local scope
       Now whenever we look into the node handle for our parameters we get local parameters
    */
    ros::NodeHandle nh("~");

    // Create parameter name and give it a default value
    std::string publisher_name = "chatter";

    // Get the parameter from the parameter server
    // If there is no "publisher" in the parameter server then nothing is returned
    nh.getParam("publisher", publisher_name);

    // Set up the publisher which is the object which sends our messages
    ros::Publisher chatter_pub = nh.advertise<std_msgs::String>(publisher_name, 5000);

    // The rate at which we 'spin', i.e how often we send data out
    ros::Rate loop_rate(10);

    int count = 0;
    while(ros::ok()) {
        // ROS uses their own String implementation, which is what we need to send data through topics
        std_msgs::String msg;

        std::stringstream ss;
        ss << "hello world" << count;
        msg.data = ss.str();
        
        // Send out the data
        chatter_pub.publish(msg);

        // Let ROS do its thing
        ros::spinOnce();

        // sleep so we dont overrun our loop rate
        loop_rate.sleep();
    }

    // We probaby never get here
    return 0;   
}


 


