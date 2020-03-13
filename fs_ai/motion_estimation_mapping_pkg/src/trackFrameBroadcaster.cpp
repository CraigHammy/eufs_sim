#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
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
        slam_sub_ = nh_.subscribe<nav_msgs::Odometry>("/slam_estimate", 1, boost::bind(&WorldFrameBroadcaster::slam_callback, this, _1));
    }

    void slam_callback(const nav_msgs::Odometry::ConstPtr& msg)
    {
        transformStamped_.header.frame_id = "track";
        transformStamped_.child_frame_id = "base_footprint";
        transformStamped_.header.stamp = ros::Time::now();

        transformStamped_.transform.translation.x = msg->pose.pose.position.x;
        transformStamped_.transform.translation.y = msg->pose.pose.position.y;
        transformStamped_.transform.translation.z = 0.0;
        
        geometry_msgs::Quaternion orient = tf::createQuaternionMsgFromYaw(msg->pose.pose.orientation.z);
        transformStamped_.transform.rotation = orient;

        tfb_.sendTransform(transformStamped_);
        //ROS_INFO("x: %f, y: %f, yaw:%f", msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.orientation.z);
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber slam_sub_;
    tf2_ros::TransformBroadcaster tfb_;
    geometry_msgs::TransformStamped transformStamped_;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "track_frame_broadcaster_node");
    ros::NodeHandle nh;
    WorldFrameBroadcaster tbc(&nh);
    ros::spin();
    return 0;
}