#include "ros/ros.h"
#include "grabber/AddTwoInts.h"
#include "grabber/GrabberCommand.h"
#include "serial/serial.h"
#include <string>

/**
 * General usage:
 * accepts commands as found in the Controller Command List at
 * https://engrauvic.engr.uvic.ca/wiki/doku.php?id=resources:equipment:grabber_controller.
 *
 * Commands are recieved as a grabber::GrabberCommand::Request, with command being the
 * 3 letter command code as found in the above table, and param being the numeric
 * argument to that command. Commands not requiring an argument, such as FWD, will silently
 * ignore any given param. Basic error checking is done on the range of param; values
 * outside the specified bounds will prevent the command from being sent to the grabber,
 * and will result in a warning being sent to the terminal and an error bit being set in the
 * response struct. Responses contain the same data as the recieved Request struct, with the
 * exception of that error bit.
 */

using namespace serial;

Serial *my_serial = nullptr;

// replaces newlines with "\n" to avoid breaking terminal output
std::string sanitize(std::string arg) {
    int index = -1;
    while((index = arg.find("\n")) != -1) {
        arg.replace(index, 1, "\\n");
    }
    return arg;
}

// needs_param is used as a second return value, on whether or not to include
// a numeric parameter with the serial output
bool checkCommand(std::string &com, unsigned long arg, bool &needs_param) {
    if(com == "\n")
    {
        needs_param = false;
        return true;
    }
    else if(com == "FWD" || com == "REV" || com == "STP")
    {
        needs_param = false;
        return true;
    }
    else if (com == "DIS" || com == "INH" || com == "DIR")
    {
        needs_param = true;
        if(arg != 0 && arg != 1)
        {
            ROS_WARN("Grabber - invalid command argument for %s: %lu", com.c_str(), arg);
            return false;
        }
        return true;
    }
    else if (com == "PWM")
    {
        needs_param = true;
        // too many digits
        if(arg > 255)
        {
            ROS_WARN("Grabber - invalid command argument for PWM: %lu", arg);
            return false;
        }
        return true;
    }
    else if (com == "SPI")
    {
        needs_param = true;
        int last_digit = arg % 10;
        if(arg > 99999999 || (last_digit != 0 && last_digit != 1))
        {
            ROS_WARN("Grabber - invalid command argument for SPI: %lu", arg);
            return false;
        }
        return true;
    }
    else if (com == "FWT" || com == "RVT")
    {
        needs_param = true;
        if(arg > 99999999)
        {
            ROS_WARN("Grabber - invalid command argument for %s: %lu", com.c_str(), arg);
        }
    }
    else
    {
        ROS_WARN("Grabber - unrecognised command: %s", com.c_str());
        return false;
    }
}

bool issueCommand(grabber::GrabberCommand::Request &req,
    grabber::GrabberCommand::Response &res)
{
    std::string output = std::string("Received grabber command: ") +
            sanitize(req.command.c_str()) + std::string(", param=") + std::to_string((unsigned long long)req.param);
    ROS_INFO(output.c_str());
    res.command = req.command;
    res.param = req.param;
    bool needs_param;
    res.success = checkCommand(res.command, res.param, needs_param);
    std::string serial_output = res.command;
    if(needs_param)
    {
        std::string param_str = std::to_string(res.param);
        if(req.command == "SPI")
            // pad with zeroes to make 8 characters
            serial_output += std::string(8 - param_str.length(), '0') + param_str;
        else
            serial_output += param_str;

    }
    if(res.success)
    {
        my_serial->write(serial_output + "\n");
    }
    return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "service_node");
  ros::NodeHandle n;

  std::string port = "/dev/pts/6";
  unsigned baud = 115200;
  my_serial = new Serial(port, baud, serial::Timeout::simpleTimeout(10000));

  ros::ServiceServer service = n.advertiseService("grabber_command", issueCommand);
  ROS_INFO("Ready to receive commands.");
  ros::spin();

  return 0;
}
