#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <nav_msgs/Odometry.h>

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

class WorldFrameBroadcaster
{
public:
    WorldFrameBroadcaster(ros::NodeHandle* nh): nh_(*nh) { initialise(); }

    void initialise()
    {
        odom_sub_ = nh_.subscribe<nav_msgs::Odometry>("/ground_truth/odom", 1, boost::bind(&WorldFrameBroadcaster::odom_callback, this, _1));
    }

    void odom_callback(const nav_msgs::Odometry::ConstPtr& msg)
    {
        transformStamped.header.frame_id = "track";
        transformStamped.child_frame_id = "base_footprint";
        transformStamped.transform.translation.x = msg->pose.pose.position.x;
        transformStamped.transform.translation.y = msg->pose.pose.position.y;
        transformStamped.transform.translation.z = 0.0;
        transformStamped.transform.rotation.x = 0.0;
        transformStamped.transform.rotation.y = 0.0;
        transformStamped.transform.rotation.z = msg->pose.pose.orientation.z;
        transformStamped.transform.rotation.w = 1.0;
        transformStamped.header.stamp = ros::Time::now();
        tfb.sendTransform(transformStamped);
        //ROS_INFO("x: %f, y: %f, yaw:%f", msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.orientation.z);
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber odom_sub_;
    tf2_ros::TransformBroadcaster tfb;
    geometry_msgs::TransformStamped transformStamped;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "track_frame_broadcaster_node");
    ros::NodeHandle nh;
    WorldFrameBroadcaster tbc(&nh);
    ros::spin();
    return 0;
}