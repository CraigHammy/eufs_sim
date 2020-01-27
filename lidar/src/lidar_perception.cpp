#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl_ros/point_cloud.h>
#include <limits>
#include <pcl/common/common.h>

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
typedef pcl::PointXYZ Point;
#define LOWERBOUNDS 0
#define UPPERBOUNDS 5
#define GPRDELTA 0.15
ros::Publisher pub;



void PCLcallback(const sensor_msgs::PointCloud2ConstPtr& input){
    //"Size Pre filter"
    ROS_INFO("POINTCLOUD RECIEVED");
    ROS_INFO("Height %d", input->height);
    ROS_INFO("Width %d", input->width);

    pcl::PCLPointCloud2 pcl_pc2; 
    PointCloud::Ptr pre(new PointCloud);
    pcl_conversions::toPCL(*input, pcl_pc2);    //Converts from ROS Point Cloud 2 to PCL Point Cloud 2
    pcl::fromPCLPointCloud2(pcl_pc2,*pre);      //Converts from PCL Point Cloud 2 to PCL Point Cloud

    PointCloud::Ptr post(new PointCloud);
    pcl::PassThrough<pcl::PointXYZ> pass;       //Create Filter
    pass.setInputCloud(pre);
    pass.setFilterFieldName("x");
    pass.setFilterLimits(LOWERBOUNDS,UPPERBOUNDS);
    pass.filter(*post);                         //Filter pre Cloud based on Above parameters

    //"Size" Post Filter
    ROS_INFO("POINTCLOUD Filtered");    
    ROS_INFO("Height %d", post->height);
    ROS_INFO("Width %d", post->width);

    Eigen::Vector4f min, max;
    pcl::getMinMax3D(*post,min,max);

    ROS_INFO("MIN %d", min);
    ROS_INFO("Lowest Point X %f", min[0]);
    ROS_INFO("Lowest Point Y %f", min[1]);
    ROS_INFO("Lowest Point Z %f", min[2]);

    ROS_INFO("MAX %d", max);
    ROS_INFO("Highest Point X %f", max[0]);
    ROS_INFO("Highest Point Y %f", max[1]);
    ROS_INFO("Highest Point Z %f", max[2]);

    PointCloud::Ptr GPR(new PointCloud);
    pcl::PassThrough<pcl::PointXYZ> gprpass;       //Create Filter
    gprpass.setInputCloud(post);
    gprpass.setFilterFieldName("z");
    gprpass.setFilterLimitsNegative(true);
    gprpass.setFilterLimits(min[2],min[2]+GPRDELTA);
    gprpass.filter(*GPR);                         //Filter pre Cloud based on Above parameters


    pcl::PCLPointCloud2 post_pc2;
    sensor_msgs::PointCloud2 output;
    pcl::toPCLPointCloud2(*GPR,post_pc2);      //Convert from PCL Point Cloud to PCL Point Cloud 2
    pcl_conversions::fromPCL(post_pc2,output);  //Convert from PCL Point Cloud 2 to ROS Point Cloud 2
    
    pub.publish(output);

}


int main(int argc, char **argv){

    ros::init(argc,argv, "PCL_Listener");

    ros::NodeHandle n; //Create Node

    ros::Subscriber sub = n.subscribe("velodyne_points", 1, PCLcallback); //Create Subscriber to velodyne 
    pub = n.advertise<PointCloud> ("FilterPoints",1); //Create Publisher for filtered cloud
    ros::spin();

    return(0);
}