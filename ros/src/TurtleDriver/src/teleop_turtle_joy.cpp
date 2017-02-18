#include <geometry_msgs/Twist.h>
#include <signal.h>
#include "ros/ros.h"
#include <stdio.h>
#include <iostream>
#include <string>
#include <sstream>
#include <cstdlib> strtol(s.c_str(),0,10());
#include "std_msgs/String.h"

class TeleopTurtle
{
public:
    TeleopTurtle();
    void chatterCallback(const std_msgs::String::ConstPtr& msg);
private:
    ros::NodeHandle nh_;
    double linear_,angular_, l_scale_, a_scale_;
    ros::Subscriber sub;

    ros::Publisher twist_pub_;

};

void TeleopTurtle::chatterCallback(const std_msgs::String::ConstPtr& msg) {
    std::string token;
    char* msgString = strdup(msg->data.c_str());
    bool dirty = false;

    ROS_INFO("Message: %s", msg->data.c_str());

    /* Axis 0 -l/+r
     * Axis 1 -f/+b
     */

    linear_ = angular_ = 0;
    token = strtok(msgString,";");
    if(token == "Axis0"){
        token = strtok(NULL,";");
         int value = std::strtol(token.c_str(),0,10);
         if (value < 0) {
            ROS_DEBUG("LEFT");
            angular_ = -1.0*value/33000;
            dirty = true;
         } else if( value > 0){
            ROS_DEBUG("RIGHT");
            angular_ = -1.0*value/33000;
            dirty = true;
         }
     }else if (token == "Axis1") {
         token = strtok(NULL,";");
         int value = std::strtol(token.c_str(),0,10);
         if (value  < 0) {
            ROS_DEBUG("FORWARD");
            linear_ = -1.0*value/33000;
            dirty = true;
         }
         else if (value > 0){
            ROS_DEBUG("REVERSE");
            linear_ = -1.0*value/33000;
            dirty = true;
         }
    }

    free(msgString);
    geometry_msgs::Twist twist;
    twist.angular.z = a_scale_*angular_;
    twist.linear.x = l_scale_*linear_;
    if(dirty == true) {
        twist_pub_.publish(twist);
        dirty = false;
    }
    return;
}


TeleopTurtle::TeleopTurtle():
    linear_(0),
    angular_(0),
    l_scale_(2.0),
    a_scale_(2.0)
{
    nh_.param("scale_param", a_scale_, a_scale_);
    nh_.param("scale_linear", l_scale_, l_scale_);

    twist_pub_ = nh_.advertise<geometry_msgs::Twist>("turtle1/cmd_vel",1);
    sub = nh_.subscribe("chatter", 1000, &TeleopTurtle::chatterCallback, this);
}

void quit(int sig){
    ros::shutdown();
    exit(0);
}

int main(int argc, char**argv){
    ros::init(argc, argv, "teleop_turtle");
    TeleopTurtle teleop_turtle;

    signal(SIGINT,quit);

    ros::spin();
    return 0;
}

