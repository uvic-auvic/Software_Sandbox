#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sstream>
#include <latency_test/dataSizeOne.h>
#include <pigpio.h>
#include <iostream>
#include <fstream>

float seconds;
uint32_t timeEnd, timeStart, timeDiff;
int count = 0;
int countGlobal = 1;
int loopRate_;

void chatterBackCb(const latency_test::dataSizeOne data)
{
    count++;
    /*if(count == loopRate_)
    {
        timeEnd = gpioTick();
    }*/
    return;
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "latency_talker");
    ros::NodeHandle nh;
    ros::Publisher chatter_one_pub = nh.advertise<latency_test::dataSizeOne>("chatter", 1000);
    ros::Subscriber chatterBackSub = nh.subscribe("chatterBack",1000, chatterBackCb);

    gpioInitialise();

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

    latency_test::dataSizeOne data;
    data.One = 32720;

    uint32_t timeGlobalStart = gpioTick();
    while(ros::ok() && countGlobal < 11)
    {
        if(count == 0)
        {
            timeStart = gpioTick();
        }
        chatter_one_pub.publish(data);
        ros::spinOnce();
        loop_rate.sleep();

        if(count >= 100000)
            break;
        /*if(count >= loopRate_)
        {
            count = 0;
            countGlobal++;
            timeDiff = timeEnd - timeStart;
            seconds = timeDiff/1000000.0;
            ROS_INFO("Time elapsed: %f sec",  seconds);
        }*/
    }

    uint32_t timeGlobalEnd = gpioTick();
    timeDiff = timeGlobalEnd - timeGlobalStart;
    seconds = timeDiff/1000000.0;
    ROS_INFO("Global time elapsed: %f", seconds);
    seconds = seconds / (count);
    ROS_INFO("Total operations: %d Average time per operation: %f s", count, seconds);
    gpioTerminate();
    return 0;
}
