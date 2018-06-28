# ROS Services

A service is similar to a topic in the sense that their is a node on either end of it, but different in function an use. Service calls are typically made less frequently, and are usually done to perform an explicit action. For example one might perform a service call to modify parameters or to have request the vision node to perform a new processing technique on an image.

# Services

A service object is required for a service to be called/requested. A single node can operate multiple services. A simple service can be found [here](/ros/src/services/src/service.cpp)

# Clients

A client object is required to ask for a service to be done. A single node can have multiple client objects make requests. A simple client can be found [here](/ros/src/services/src/client.cpp)

# Important Note about ROS Msgs

This example won't compile as is, unless you have the ROS beginner tutorials installed. You can modify it to work by removing the dependency on the beginner messages and [creating your own service msg](/ros/src/msg/).

# ROS Wiki Tutorial

[Take a look at the example here for more detail](http://wiki.ros.org/ROS/Tutorials/WritingServiceClient%28c%2B%2B%29)
