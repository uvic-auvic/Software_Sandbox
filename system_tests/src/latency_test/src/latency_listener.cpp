#include <ros/ros.h>
#include <latency_test/dataSizeOne.h>
#include <time.h>
#include <iostream>
#include <fstream>
#include <stdio.h>

int count = 0;
int countGlobal = 1;
long int timeStart, timeEnd, timeDiff, timeGlobalStart, timeGlobalEnd;
double seconds;
bool startFlag = false;
int loopRate_;
struct timespec gettime_now;


void chatterCb(const latency_test::dataSizeOne data)
{
    if (count == 0)
    {
        if(startFlag == false)
        {
            clock_gettime(CLOCK_REALTIME, &gettime_now);
            timeGlobalStart = gettime_now.tv_nsec;
            startFlag = true;
        }
        clock_gettime(CLOCK_REALTIME, &gettime_now);
        timeStart = gettime_now.tv_nsec;
    }
    count++;

    clock_gettime(CLOCK_REALTIME, &gettime_now);
    if((gettime_now.tv_nsec - timeStart) < 0)
        timeEnd += 1000000000;
    if((gettime_now.tv_nsec - timeGlobalStart) < 0)
        timeGlobalEnd += 1000000000;
    if(count == loopRate_)
    {
        clock_gettime(CLOCK_REALTIME, &gettime_now);
        timeEnd += gettime_now.tv_nsec;
        count = 0;
        countGlobal++;
        timeDiff = timeEnd - timeStart;
        timeEnd = 0;
        seconds = timeDiff/1000000000.0;
        ROS_INFO("Time elapsed for %d bytes: %lf sec", countGlobal, seconds);
        if(countGlobal == 32)
        {
            clock_gettime(CLOCK_REALTIME, &gettime_now);
            timeGlobalEnd += gettime_now.tv_nsec;
            timeDiff = timeGlobalEnd - timeGlobalStart;
            seconds = timeDiff/1000000000.0;
            ROS_INFO("Global time elapsed: %lf sec", seconds);
            seconds = seconds / countGlobal;
            ROS_INFO("Exoected average time: 0.5 sec. Actual average time: %lf sec", seconds);
        }
    }

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
    timeEnd = 0;
    timeGlobalEnd = 0;
    ros::init(argc, argv, "latency_listener");
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe("chatter", 1000, chatterCb);
    ros::spin();

    return 0;
}
