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

#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <visualization_msgs/Marker.h>


typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
typedef pcl::PointXYZ Point;

#define LOWERBOUNDS 0
#define UPPERBOUNDS 5
#define GPRDELTA 0.15
ros::Publisher pub;


void PCLcallback(const sensor_msgs::PointCloud2ConstPtr& input){

    //Input Cloud
    ROS_INFO("POINTCLOUD RECIEVED");
    // ROS_INFO("Height %d", input->height);
    // ROS_INFO("Width %d", input->width);

	//Convert to usable form
    pcl::PCLPointCloud2 pcl_pc2; 
    PointCloud::Ptr pre(new PointCloud);
    pcl_conversions::toPCL(*input, pcl_pc2);    //Converts from ROS Point Cloud 2 to PCL Point Cloud 2
    pcl::fromPCLPointCloud2(pcl_pc2,*pre);      //Converts from PCL Point Cloud 2 to PCL Point Cloud

	//Begin FOV Trimming
    PointCloud::Ptr post(new PointCloud);
    pcl::PassThrough<pcl::PointXYZ> pass;       //Create Filter
    pass.setInputCloud(pre);
    pass.setFilterFieldName("x");
    pass.setFilterLimits(LOWERBOUNDS,UPPERBOUNDS);
    pass.filter(*post);                         //Filter pre Cloud based on Above parameters ()

    // ROS_INFO("POINTCLOUD Filtered");    
    // ROS_INFO("Height %d", post->height);
    // ROS_INFO("Width %d", post->width);
    //Finished FOV Trimming

	//Begins Ground Plane Removal
    Eigen::Vector4f min, max; //Vectors to hold max and min points
    pcl::getMinMax3D(*post,min,max);

    // ROS_INFO("MIN %d", min);
    // ROS_INFO("Lowest Point X %f", min[0]);
    // ROS_INFO("Lowest Point Y %f", min[1]);
    // ROS_INFO("Lowest Point Z %f", min[2]);

    // ROS_INFO("MAX %d", max);
    // ROS_INFO("Highest Point X %f", max[0]);
    // ROS_INFO("Highest Point Y %f", max[1]);
    // ROS_INFO("Highest Point Z %f", max[2]);

    PointCloud::Ptr GPR(new PointCloud);
    pcl::PassThrough<pcl::PointXYZ> gprpass;       //Create Filter
    gprpass.setInputCloud(post);
    gprpass.setFilterFieldName("z");
    gprpass.setFilterLimitsNegative(true);
    gprpass.setFilterLimits(min[2],min[2]+GPRDELTA);
    gprpass.filter(*GPR);                         //Filter pre Cloud based on Above parameters
    //Finished Ground Plane Removal

	//Begin Clustering
    pcl::search::KdTree<Point>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(GPR);

    std::vector<pcl::PointIndices> cluster_indices;

	pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
	ec.setClusterTolerance(0.3); // 10cm
	ec.setMinClusterSize(3);
	ec.setMaxClusterSize(40);
	ec.setSearchMethod(tree);
	ec.setInputCloud(GPR);
	ec.extract(cluster_indices);
	
	pcl::PCDWriter writer;
	int j = 0;
	for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
	{
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
		for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
		cloud_cluster->points.push_back (GPR->points[*pit]); //*
		cloud_cluster->width = cloud_cluster->points.size ();
		cloud_cluster->height = 1;
		cloud_cluster->is_dense = true;

		std::cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size () << " data points." << std::endl;
		std::stringstream ss;
		ss << "cloud_cluster_" << j << ".pcd";
		writer.write<pcl::PointXYZ> (ss.str (), *cloud_cluster, false); //*
		j++;
	}


	//Convert Back to ROS usable format
    pcl::PCLPointCloud2 post_pc2;
    sensor_msgs::PointCloud2 output;
    pcl::toPCLPointCloud2(*GPR,post_pc2);      //Convert from PCL Point Cloud to PCL Point Cloud 2
    pcl_conversions::fromPCL(post_pc2,output);  //Convert from PCL Point Cloud 2 to ROS Point Cloud 2
    
	//Published Filtered Cloud
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