#include <ros/ros.h>
#include <latency_test/dataSizeOne.h>
#include <time.h>
#include <iostream>
#include <fstream>
#include <stdio.h>

ros::Publisher pub;
int count = 0;
int loopRate_;

void chatterCb(latency_test::dataSizeOne data)
{

    pub.publish(data);

    return;
}

int main(int argc, char** argv)
{
    std::ifstream latencyConfigFile;
    latencyConfigFile.open("/home/ubuntu/ROV02/src/latency_test/include/latency_test/latencyConfig.txt");
    if(latencyConfigFile.is_open())
    {
        std::string line;
        char* lineChar;
        while(getline(latencyConfigFile, line)){
            lineChar = (char*)line.c_str();
            line = strtok(lineChar, ";");
            if(line == "loopRate_"){
                    line = strtok(NULL,";");
                    loopRate_ = strtol(line.c_str(),0,10);
            }
            else
            {
                ROS_INFO("Unknown config input: %s", lineChar);
            }
        }
        ROS_INFO("Latency Configuration complete.");
    }
    else
    {
        ROS_INFO("Latency Configuration file could not be found/opened... Shutting node off.");
    }
    latencyConfigFile.close();

    ros::init(argc, argv, "latency_listener");
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe("chatter", 1000, chatterCb);
    pub = nh.advertise<latency_test::dataSizeOne>("chatterBack",1000);
    ros::spin();

    return 0;
}
