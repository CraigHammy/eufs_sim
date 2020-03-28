#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>
#include "montecarlo_localisation.hpp"
#include <nav_msgs/OccupancyGrid.h>
#include <Eigen/Core>
#include <Eigen/Dense>

class Testing
{
public:
    Testing(ros::NodeHandle* nh): private_nh_(nh) 
    { 
        ROS_WARN("constructor");
        start = false;
        start2 = false;
        boost::shared_ptr<nav_msgs::OccupancyGrid> map_ptr(new nav_msgs::OccupancyGrid(map));
        MCL mcl(private_nh_, map_ptr);
        mcl2 = mcl;
        scansub = private_nh_->subscribe<sensor_msgs::LaserScan>("/laserscan", 1, boost::bind(&Testing::scanCallback, this, _1));
        odomsub = private_nh_->subscribe<nav_msgs::Odometry>("/ground_truth/odom", 1, boost::bind(&Testing::odomCallback, this, _1));
        mapsub = private_nh_->subscribe<nav_msgs::OccupancyGrid>("/map", 1, boost::bind(&Testing::mapCallback, this, _1));
        ROS_WARN("initialised class");
    };

    void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg);
    void scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg);
    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);


private:
    MCL mcl2;
    ros::Subscriber scansub;
    ros::Subscriber odomsub;
    ros::Subscriber mapsub;

    ros::NodeHandle* private_nh_;
    std::vector<float> scan;
    Eigen::Vector3f xEst;
    bool start;
    bool start2;
    nav_msgs::OccupancyGrid map;
};


void Testing::mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
    ROS_WARN("map callback");
    map = *msg;
    if (start and start2)
        mcl2.amcl_estimation_step(xEst, scan);
}

void Testing::scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    ROS_WARN("scan callback");
    start2 = true;
    scan = msg->ranges;
    //if (start == true)
    //    mcl2.amcl_estimation_step(xEst, scan);
}

void Testing::odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    ROS_WARN("odom callback");
    start = true;
    xEst(0) = msg->pose.pose.position.x;
    xEst(1) = msg->pose.pose.position.y;

    double roll, pitch, yaw;
    tf::Quaternion q(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
    tf::Matrix3x3 m(q);
    m.getRPY(roll, pitch, yaw);
    xEst(2) = yaw;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "testing_node");
    ros::NodeHandle nh("~");
    ROS_WARN("node created");
    Testing t(&nh);
    ros::spin();
}