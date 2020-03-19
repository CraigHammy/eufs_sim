#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>

class Testing {
public:
    double imu_yaw_, odom_yaw_;
    Testing(ros::NodeHandle* nh): nh_(*nh), imu_yaw_(0.0), odom_yaw_(0.0) 
    {
        imu_sub_ = nh_.subscribe<sensor_msgs::Imu>("/imu", 1, boost::bind(&Testing::imu_cb, this, _1));
        odom_sub_ = nh_.subscribe<nav_msgs::Odometry>("/ground_truth/odom", 1, boost::bind(&Testing::odom_cb, this, _1));
    }
    void imu_cb(const sensor_msgs::Imu::ConstPtr& msg)
    {
        tf::Quaternion q(msg->orientation.x, msg->orientation.y,
            msg->orientation.z, msg->orientation.w);
        tf::Matrix3x3 m(q);
        double pitch, roll;
        m.getRPY(roll, pitch, imu_yaw_);
        ROS_INFO("IMU YAW: %f", imu_yaw_);
    }

    void odom_cb(const nav_msgs::Odometry::ConstPtr& msg)
    {
        tf::Quaternion q(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
            msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
        tf::Matrix3x3 m(q);
        double pitch, roll;
        m.getRPY(roll, pitch, odom_yaw_);
        ROS_INFO("ODOM YAW: %f", odom_yaw_);
    }

    void geodetic_to_enu();

private:
    ros::NodeHandle nh_;
    ros::Subscriber odom_sub_;
    ros::Subscriber imu_sub_;
};

void Testing::geodetic_to_enu()
{
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "test_node");
    ros::NodeHandle n;
    Testing t(&n);
    ros::spin();
}