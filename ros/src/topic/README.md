# ROS Topic Example

As mentioned previously, Topics are a way to transmit/recieve a continuous stream of data. On one end you have subscribers funnelling data into a topic and on the other end you have subscribers recieving and handling the data. This mechanism provides separation of the 2 entities and ensures their logic doesn't get mixed up.

# Publishers

You can have multiple publishers all publishing to a topic. A simple publisher can be found [here](/ros/src/topic/src/publisher.cpp)

# Subscribers

You can have multiple subscribers subscribed to a topic. A simple subscriber can be found [here](/ros/src/topic/src/subscriber)
# ROS Wiki Tutorial

[Take a look at the example here for more detail](http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28c%2B%2B%29)
