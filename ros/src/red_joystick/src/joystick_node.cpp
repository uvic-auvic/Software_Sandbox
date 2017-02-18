#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>
#include "../include/red_joystick/joystick.cpp"
#include <unistd.h>
#include <iostream>


int main(int argc, char **argv){
    ros::init(argc, argv, "joystick_node");
    ros::NodeHandle n;
    ros::Publisher chatter_pub = n.advertise<std_msgs::String>("/input/joystick", 10);
    ros::Publisher GUI_Pub = n.advertise<std_msgs::String>("/GUI/general", 1);
    ros::Rate loop_rate(1000);
    std_msgs::String msg;
    std::stringstream guiss;
    Joystick joystick("/dev/input/js0");
    usleep(1000000);
    if(!joystick.isFound())
    {

        guiss << "Joystick not found...";
        msg.data = guiss.str();
        ROS_INFO("%s", msg.data.c_str());
        GUI_Pub.publish(msg);
        while(!joystick.isFound())
        {
            joystick.openPath("/dev/input/js0");
            usleep(1000000);
            ros::spinOnce();
        }
        guiss.str(std::string());
        guiss.clear();
    }

    guiss << "Joystick found";
    msg.data = guiss.str();
    ROS_INFO("%s", msg.data.c_str());
    GUI_Pub.publish(msg);
    ros::spinOnce();

    int count = 0;
    while(ros::ok()) {
        std::stringstream ss;
        JoystickEvent event;
        bool eventFlag = false;

        if(joystick.sample(&event)){
            int eventNumber = event.number;
            int eventValue = event.value;
            if(event.isButton()){
                ss << "Button;" << eventNumber << ";" << eventValue;
                eventFlag = true;
            }
            else if(event.isAxis()){
                ss << "Axis;" << eventNumber << ";" << eventValue;
                eventFlag = true;
            }
        }

        if(eventFlag == true){
            msg.data = ss.str();
            ROS_INFO("%s", msg.data.c_str());

            chatter_pub.publish(msg);
            eventFlag = false;
        }

        ros::spinOnce();
        loop_rate.sleep();
        ++count;
    }
    ROS_INFO("END");
    return 0;
}
