# Launchfiles

There are 2 types of launchfiles that we use. Strictly speaking they do the same thing, but we use them in a hierarchical fashion to organize things.

# Local Launchfile

The local launchfile is reponsible for providing a package with default parameters. They are there so that in the event that we want to test the whole system, we do not need to know all the parameters required for every node to function properly, we can simply call upon the local launchfile and it will handle the rest. An example of a launchfile can be found [here](/ros/src/launchfiles/launchfiles/local.launch).

# Global Launchfile

The global launchfile is the file which will launch the entire system. ROS launchfiles have the ability to 'inlcude' other launchfiles and inherit their properties, such as all the nodes they launch and arguments they contain. The global launch file can then override these properties so that we can initiate the system with our own paramters. An example of why this is useful is to imagine the scenario where we are testing our system model in Gazebo. We do not want the real motor controller to start up and start issuing commands. It will fail the second it attempts to identify the serial device. Instead we can pass in a parameter at runtime informing motor_controller node that all it's motor directives should be directed at the virtual motors in Gazebo. For an example of a roslaunch file, take a look [here](/ros/src/launchfiles/global.launch).

# Miscellaneous Operations

# External Links

http://wiki.ros.org/roslaunch
