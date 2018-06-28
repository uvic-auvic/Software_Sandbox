# MSG and SRV Files

Message and Service files are small text files which are used to autogenerate boilerplate class files which contain nothing but data handling, i.e class files which are used to formalize how data is read, created and sent. 

# MSG files

MSG contain a member declaration on each line e.g.

```
string first_name
string last_name
int age
int score
```

When the project is compiled this will autogenerate into C++ class code which can be read and modified by passing the msg across topics. The usual data types are all supported, a list of which can be found [here](http://wiki.ros.org/msg). MSGs are stored in a directory called `msg` with a .msg extenstion.

# SRV Files

Service files are used to generate a bit more complex code, all of which is hidden away and we dont really need to worry about. If you are familiar with [services](/ros/src/services) you will notice that in the [service function declaration](/ros/src/services/src/service.cpp) it takes 2 arguments, a request and a response. A request is the input parameters that the service will take, whereas the response are the paramters that the service function will return.

```
string request_param_1
int    request_param_2
---
string response_param_1
int    response_param_2
```

As you can see the request and response parameters are separated, with the request on the top and the response on the bottom. The triple dash is necessary. The srv file is stored in a directory called `srv` with a .srv extenstion.

# Modifying CMakeLists.txt and package.xml

In order for ROS to know what it needs to generate, your CMakeLists.txt and package.xml have to be told that they support message generation.

ensure the following sections are added into your CMakeLists.txt

```
find_package(catking REQUIRED COMPONENTS
    roscpp
    rospy
    std_msgs
    message_generation
)
```

```
catkin_package(
    ...
    CATKIN_DEPENDS message_runtime ...
    ...)
```

```
add_message_files(
    FILES
    filename.msg # obviously replace this with your filename
)
```

```
generate_messages(
    DEPENDENCIES
    std_msgs
)
```

# External Links

http://wiki.ros.org/ROS/Tutorials/CreatingMsgAndSrv