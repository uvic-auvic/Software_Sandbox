#include <ros/ros.h>
#include <serial/serial.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "serialtest_node");
    ros::NodeHandle nh("~");

    std::string port;
    nh.getParam("port", port);

    serial::Serial conn(port,115200,serial::Timeout::simpleTimeout(1000));
    if(!conn.isOpen())
    {
        ROS_ERROR("Serial failed to open");
        return 1;
    }

    ros::Rate loop_rate(5);
    while(ros::ok()) {
        conn.write("HELL");
        std::string response = conn.readline(1000, "\n");
        ROS_INFO("Rx from Serial Device: '%s'", response.c_str());
        loop_rate.sleep();
    }
    return 0;
}
