#include <ros/ros.h>
#include <serial/serial.h>
#include <unistd.h>
#include <stdio.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "serialtest_node");
    ros::NodeHandle nh_;
    time_t timer, starttimer;
    std::string teststring;
    serial::Serial serialTestSerial("/dev/ttyAMA0",115200,serial::Timeout::simpleTimeout(100));
    if(!serialTestSerial.isOpen())
    {
        std::cout << "Serial failed to open.";
    }
    ros::Rate loop_rate(5);
    int count = 0;
    float timevpy;
    time(&starttimer);
    while(ros::ok())
    {
        std::stringstream ss;
        ss << "hello world" << count;
        serialTestSerial.write(ss.str());
        serialTestSerial.read(teststring);
        time(&timer);
        timevpy = difftime(timer, starttimer);
        ROS_INFO("Count: %d", count);
        ROS_INFO("Time: %f", timevpy);
        ros::spinOnce();
        loop_rate.sleep();
        ++count;
    }
    ros::shutdown();
    return 0;
}
