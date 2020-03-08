#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>

/*
In relation to a body the standard is:

x forward
y left
z up

Suffix Frames
In the case of cameras, there is often a second frame defined with a "_optical" suffix. This uses a slightly different convention:

z forward
x right
y down
*/

int main(int argc, char** argv)
{
    ros::init(argc, argv, "track_frame_broadcaster_node");
    ros::NodeHandle nh;
    tf2_ros::TransformBroadcaster tfb;
    geometry_msgs::TransformStamped transformStamped;

    transformStamped.header.frame_id = "track";
    transformStamped.child_frame_id = "odom";
    transformStamped.transform.translation.x = 0.0;
    transformStamped.transform.translation.y = 0.0;
    transformStamped.transform.translation.z = 0.0;
    tf2::Quaternion q;
    q.setRPY(0, 0, 0);
    transformStamped.transform.rotation.x = q.x();
    transformStamped.transform.rotation.y = q.y();
    transformStamped.transform.rotation.z = q.z();
    transformStamped.transform.rotation.w = q.w();

    ros::Rate rate(10.0);
    while (nh.ok()){
        transformStamped.header.stamp = ros::Time::now();
        tfb.sendTransform(transformStamped);
        rate.sleep();
        }
    return 0;
}