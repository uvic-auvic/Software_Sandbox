#include <ros/ros.h>
#include "std_msgs/String.h"
#include <signal.h>
#include <unistd.h>
#include <termios.h>
#include <stdio.h>
#include <ros/console.h>
#include <string.h>

//KB Code Defines
/*
#define KEYCODE_NUM0    0x60
#define KEYCODE_NUM1    0x61
#define KEYCODE_NUM2    0x62
#define KEYCODE_NUM3    0x63
#define KEYCODE_NUM4    0x64
#define KEYCODE_NUM5    0x65
#define KEYCODE_NUM6    0x66
#define KEYCODE_NUM7    0x67
#define KEYCODE_NUM8    0x68
#define KEYCODE_NUM9    0x69
#define KEYCODE_NUMDIV  0x6F
#define KEYCODE_NUMMUL  0x6A
#define KEYCODE_SUB     0x6D
#define KEYCODE_ADD     0x6B
#define KEYCODE_NUMDOT  0x6E
*/

#define KEYCODE_0       0x30
#define KEYCODE_1       0x31
#define KEYCODE_2       0x32
#define KEYCODE_3       0x33
#define KEYCODE_4       0x34
#define KEYCODE_5       0x35
#define KEYCODE_6       0x36
#define KEYCODE_7       0x37
#define KEYCODE_8       0x38
#define KEYCODE_9       0x39
#define KEYCODE_DIV     0x2F
#define KEYCODE_MUL     0x2A
#define KEYCODE_ADD     0x2B
#define KEYCODE_SUB     0x2D
#define KEYCODE_DOT     0x2E

//Globals
int kfd = 0;
struct termios cooked, raw;


void quit(int sig)
{
    (void)sig;
    tcsetattr(kfd, TCSANOW, &cooked);
    ROS_INFO("Shutting down");
    ros::shutdown();
    exit(0);
}

int main(int argc, char **argv)
{
    //Variables
    char KB_char = '\0';

    //Actual Code
    ros::init(argc,argv,"rov_keyboard");
    ros::NodeHandle n;
    ros::Rate loop_rate(1);
    std_msgs::String msg;
    std::stringstream guiss;

    signal(SIGINT,quit);

    ROS_INFO("KEYBOARD INPUT NODE ONLINE");

    //Get the console in raw mode
    tcgetattr(kfd, &cooked);
    memcpy(&raw, &cooked, sizeof(struct termios));
    raw.c_lflag &=~ (ICANON | ECHO);
    // Setting a new line, then end of file
    raw.c_cc[VEOL] = 1;
    raw.c_cc[VEOF] = 2;
    tcsetattr(kfd, TCSANOW, &raw);

    while(ros::ok())
    {
        //ROS_INFO("BOOP");
        //ros::spinOnce();

        // get the next event from the keyboard
        if(read(kfd, &KB_char, 1) < 0)
        {
          ROS_ERROR("read():");
          exit(-1);
        }

        //ROS_DEBUG("value: 0x%02X\n", KB_char);
        ROS_INFO("Code: %c %x", KB_char, KB_char);

        switch(KB_char)
        {
            case KEYCODE_0:
                ROS_INFO("LEFT");
                break;

            default:
                ROS_DEBUG("Code: %c", KB_char);
                break;
        }
        ros::spinOnce();
        //loop_rate.sleep();
    }

    return(0);
}
