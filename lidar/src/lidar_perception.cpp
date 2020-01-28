//C Includes
#include <limits>
#include <string>
//ROS Inlcudes
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
//PCL Includes
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl/ModelCoefficients.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/common/common.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
//PCL ROS Includes
#include <pcl_ros/transforms.h>
#include <pcl_ros/point_cloud.h>

//typedef's for longer or common variable types
typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
typedef pcl::PointXYZ Point;

//FOV Trimming Parameters
#define LOWERBOUNDS 0
#define UPPERBOUNDS 5
//Ground Plane Removal Parameters
#define GPRDELTA 0.15

//Clustering Parameters
#define CLUSTERTOLERANCE 0.3
#define MINCLUSTERSIZE 2
#define MAXCLUSTERSIZE 20

//Bounding Box Scalers
#define BOUNDINGBOXSIZESCALER 3
#define BOUNDINGBOXHEIGHTSCALER 1
#define MINIMUMBOXSIZE 0.08
//Debug Parameters
//#define DEBUGOUTPUT
#define ALLCLOUDS

//Global publisher variable
ros::Publisher fovTrimPub;
ros::Publisher groundPlaneRemovalPub;
ros::Publisher markerPub;
ros::Publisher restoredPub;

//Filter Method
void passThroughFilter(PointCloud::Ptr inputCloud, std::string filterField, float lowerLimit, float upperLimit, PointCloud::Ptr output, bool negative){
    pcl::PassThrough<pcl::PointXYZ> pass;       //Create Filter
    pass.setInputCloud(inputCloud);
    pass.setFilterFieldName(filterField);
    pass.setFilterLimits(lowerLimit,upperLimit);
    if(negative){
        pass.setFilterLimitsNegative(true);
    }
    pass.filter(*output);   //Filter Cloud based on Above parameters ()

}

//Convert Down Method
void ConvertToPCL(sensor_msgs::PointCloud2ConstPtr input, PointCloud::Ptr output){

    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(*input, pcl_pc2);    //Converts from ROS Point Cloud 2 to PCL Point Cloud 2
    pcl::fromPCLPointCloud2(pcl_pc2,*output);      //Converts from PCL Point Cloud 2 to PCL Point Cloud

}

//Convert Up Method
sensor_msgs::PointCloud2 ConvertFromPCL(PointCloud::Ptr  input){

    sensor_msgs::PointCloud2 output;
    pcl::PCLPointCloud2 pcl_pc2;
    pcl::toPCLPointCloud2(*input,pcl_pc2);      //Convert from PCL Point Cloud to PCL Point Cloud 2
    pcl_conversions::fromPCL(pcl_pc2,output);  //Convert from PCL Point Cloud 2 to ROS Point Cloud 2

    return output;
}

//Bounding Box Method(Center + Size)
void getBoundingBox(PointCloud::Ptr inputCloud,geometry_msgs::Pose* pose, geometry_msgs::Vector3* dimensions){

    Eigen::Vector4f min, max; //Vectors to hold max and min points
    pcl::getMinMax3D(*inputCloud,min,max); //Get max and min points
    
    //Determine the Position of the box based on the mid point of the min and max value
    pose->position.x = (max.x() + min.x())/2;
    pose->position.y = (max.y() + min.y())/2;
    pose->position.z = (max.z() + min.z())/2;

    //Determine the larger size of the bounding box 
    float xDiff = max.x() - min.x();
    float yDiff = max.y() - min.y();
    float xySize = 0;

    if (xDiff>yDiff){
        xySize = xDiff;
    }else{
        xySize = yDiff;
    }

    if(xySize< MINIMUMBOXSIZE){
        xySize = MINIMUMBOXSIZE;
    }

    //Assign bounding box size to message 
    dimensions->x =  xySize*BOUNDINGBOXSIZESCALER;
    dimensions->y = xySize*BOUNDINGBOXSIZESCALER;
    dimensions->z = BOUNDINGBOXHEIGHTSCALER;

    ROS_INFO("BOX X %f", dimensions->x);
    ROS_INFO("BOX Y %f", dimensions->y);
    ROS_INFO("BOX Z %f", dimensions->z);

}

//Covnert Bounding Box Styles
void coverttoPCLBox(Eigen::Vector4f& cbmin, Eigen::Vector4f& cbmax, geometry_msgs::Pose pose, geometry_msgs::Vector3 dimensions){

    cbmin[0] = pose.position.x - (dimensions.x/2);
    cbmin[1] = pose.position.y - (dimensions.y/2);
    cbmin[2] = pose.position.z - (dimensions.z/2);
    cbmin[3] = 1.0;

    cbmax[0] = pose.position.x + (dimensions.x/2);
    cbmax[1] = pose.position.y + (dimensions.y/2);
    cbmax[2] = pose.position.z + (dimensions.z/2);
    cbmax[3] = 1.0;
    
}
//Clustering Method

void PCLcallback(const sensor_msgs::PointCloud2ConstPtr& input){

    //Input Cloud
    ROS_INFO("POINTCLOUD RECIEVED");

	//Convert to usable form
    PointCloud::Ptr pre(new PointCloud);
    ConvertToPCL(input,pre);

	//Begin FOV Trimming
    PointCloud::Ptr post(new PointCloud);
    passThroughFilter(pre, "x", LOWERBOUNDS, UPPERBOUNDS, post, false);
    //Finished FOV Trimming

	//Begins Ground Plane Removal
    Eigen::Vector4f min, max; //Vectors to hold max and min points
    pcl::getMinMax3D(*post,min,max);

    PointCloud::Ptr gpr(new PointCloud);
    passThroughFilter(post, "z", min[2],min[2]+GPRDELTA, gpr, true);
    //Finished Ground Plane Removal

	//Begin Clustering
    pcl::search::KdTree<Point>::Ptr tree (new pcl::search::KdTree<Point>);
    tree->setInputCloud(gpr);

    std::vector<pcl::PointIndices> cluster_indices;

	pcl::EuclideanClusterExtraction<Point> ec;
	ec.setClusterTolerance(CLUSTERTOLERANCE); // 30cm
	ec.setMinClusterSize(MINCLUSTERSIZE);
	ec.setMaxClusterSize(MAXCLUSTERSIZE);
	ec.setSearchMethod(tree);
	ec.setInputCloud(gpr);
	ec.extract(cluster_indices);
    //Finished Clustering

	int j = 0;
	visualization_msgs::MarkerArray markers;
	for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
	{
		PointCloud::Ptr cloud_cluster (new PointCloud);
		for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
		cloud_cluster->points.push_back(gpr->points[*pit]); 
		cloud_cluster->width = cloud_cluster->points.size();
		cloud_cluster->height = 1;
		cloud_cluster->is_dense = true;

        //Create Marker Box for Cone
        visualization_msgs::Marker object_marker;
        object_marker.ns = "cluster";
        object_marker.id = j;
        object_marker.header.frame_id="velodyne";
        object_marker.type = visualization_msgs::Marker::CUBE;
        getBoundingBox(cloud_cluster, &object_marker.pose, &object_marker.scale);
        object_marker.color.g=1;
        object_marker.color.a=0.3;
		markers.markers.push_back(object_marker);
        //Using Marker Variables heavily ties two parts together conside reformating in future

        //Begin Cone Restoration
        pcl::CropBox<Point> cb;
		cb.setInputCloud(pre);
        Eigen::Vector4f cbmin, cbmax; //Vectors to hold max and min points
        coverttoPCLBox(cbmin,cbmax, object_marker.pose, object_marker.scale);
		cb.setMax(cbmax);
		cb.setMin(cbmin);
		PointCloud::Ptr croppedCloud(new PointCloud);
		cb.filter(*croppedCloud);
        //Finished Cone Restoration

        //Publish PointCloud containing restored cone
		sensor_msgs::PointCloud2 restoredCone;
		restoredCone = ConvertFromPCL(croppedCloud);
		restoredPub.publish(restoredCone);
        

		//End of Statement
		j++;
	}

    //Publisher marker Array (Bounding Boxes of cone candidates)
	ROS_INFO("Marker Array Size: %d", markers.markers.size());
	markerPub.publish(markers);

    #ifdef ALLCLOUDS 

    sensor_msgs::PointCloud2 fovTrimmed;
    fovTrimmed = ConvertFromPCL(post);
    fovTrimPub.publish(fovTrimmed);

    sensor_msgs::PointCloud2 gprRemoved;
    gprRemoved = ConvertFromPCL(gpr);
    groundPlaneRemovalPub.publish(gprRemoved);

    #endif

}


int main(int argc, char **argv){

    ros::init(argc,argv, "PCL_Listener");

    ros::NodeHandle n; //Create Node

    //Subscribers
    ros::Subscriber sub = n.subscribe("velodyne_points", 1, PCLcallback); //Create Subscriber to velodyne
    
    //Publishers
    fovTrimPub = n.advertise<PointCloud>("fov_trimmed_cloud",1); //Create Publisher for trimmed cloud
    groundPlaneRemovalPub = n.advertise<PointCloud>("gpr_cloud",1); //Create Publisher for GPR cloud
    markerPub = n.advertise<visualization_msgs::MarkerArray>("cone_markers",1); //Create Publisher for Marker array
	restoredPub = n.advertise<PointCloud>("restored_cones",5); //Create Publisher for restored cones cloud
    ros::spin();

    return(0);
}