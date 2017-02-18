#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <turtlesim/Pose.h>


int main(int argc, char** argv)
{
    ros::init(argc, argv, "camera_tf_broadcaster");


    ros::NodeHandle node;

    static tf::TransformBroadcaster br;

    tf::Transform transform;
    ros::Rate loop_rate(10.0);

    while(ros::ok())
    {
        transform.setOrigin(tf::Vector3(0.0, -0.25, 0.750) );

        transform.setRotation( tf::Quaternion(0, 0, 0, 1) );
        br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "googly", "camera1"));
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
