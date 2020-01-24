#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
//#include <pcl/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

void PCLcallback(const sensor_msgs::PointCloud2ConstPtr& input){
    ROS_INFO("POINTCLOUD RECIEVED");
    ROS_INFO("Height %d", input->height);
    ROS_INFO("Width %d", input->width);

    pcl::PassThrough<pcl::PointXYZ> pass;
    

}

int main(int argc, char **argv){

    ros::init(argc,argv, "PCL_Listener");

    ros::NodeHandle n;

    ros::Subscriber sub = n.subscribe("velodyne_points", 1, PCLcallback);

    ros::spin();

    return(0);
}