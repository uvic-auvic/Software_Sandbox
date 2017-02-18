#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sstream>
#include <latency_test/dataSizeOne.h>
#include <pigpio.h>
#include <iostream>
#include <fstream>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "latency_talker");
    ros::NodeHandle nh;
    ros::Publisher chatter_one_pub = nh.advertise<latency_test::dataSizeOne>("chatter", 1000);


    gpioInitialise();


    int loopRate_;

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


    ros::Rate loop_rate(loopRate_);

    int count = 0;
    int countGlobal = 1;
    uint32_t timeGlobalStart = gpioTick();
    latency_test::dataSizeOne data;
    data.One = 1;

    uint32_t timeEnd;
    uint32_t timeStart = gpioTick();
    uint32_t timeDiff;
    float seconds;
    while(ros::ok() && countGlobal < 32)
    {
        if(count == 0)
        {
            timeStart = gpioTick();
        }
        chatter_one_pub.publish(data);
        ros::spinOnce();
        loop_rate.sleep();
        count++;

        if(count == loopRate_)
        {
            data.One = data.One << 1;
            count = 0;
            countGlobal++;
            timeEnd = gpioTick();
            timeDiff = timeEnd - timeStart;
            seconds = timeDiff/1000000.0;
            ROS_INFO("Time elapsed for %d bytes: %f sec", countGlobal, seconds);
        }
    }

    uint32_t timeGlobalEnd = gpioTick();
    timeDiff = timeGlobalEnd - timeGlobalStart;
    seconds = timeDiff/1000000.0;
    ROS_INFO("Global time elapsed: %f", seconds);
    seconds = seconds / countGlobal;
    ROS_INFO("Average time: %f", seconds);
    return 0;
}
