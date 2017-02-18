#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <turtlesim/Pose.h>

void poseCallback(const turtlesim::PoseConstPtr& msg)
{
    static tf::TransformBroadcaster br;

    tf::Transform transform;
    transform.setOrigin(tf::Vector3(msg->x, msg->y, msg->theta) );

    transform.setRotation( tf::Quaternion(0, 0, 0, 1) );
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "camera1", "video1"));
    return;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "camera_tf_broadcaster");

    ros::NodeHandle node;
    ros::Subscriber sub = node.subscribe("camera1/pose", 10, &poseCallback);

    ros::spin();
    return 0;
}
