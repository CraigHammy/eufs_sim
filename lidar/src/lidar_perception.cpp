//C Includes
#include <limits>
#include <string>
#include <math.h>
#include<iostream>
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
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/common.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/crop_box.h>
#include <pcl/kdtree/kdtree.h>
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
#define GPRX 2.0
#define GPRY 1.0
#define GPRDELTA 0.15

//Clustering Parameters
#define CLUSTERTOLERANCE 0.3
#define MINCLUSTERSIZE 1
#define MAXCLUSTERSIZE 20

//Bounding Box Parameters
#define BOUNDINGBOXSIZESCALER 3
#define BOUNDINGBOXHEIGHTSCALER 1
#define MINIMUMBOXSIZE 0.08

//Debug Parameters
#define ALLSTAGES

//Global Publisher creation
#ifdef ALLSTAGES
	ros::Publisher fovTrimPub;
	ros::Publisher groundPlaneRemovalPub;
	ros::Publisher markerPub;
	ros::Publisher GPRmarkerPub;
#endif
ros::Publisher restoredPub;

//Filter Method
void passThroughFilter(PointCloud::Ptr input, std::string filterField, float lowerLimit, float upperLimit, PointCloud::Ptr output, bool negative){
    pcl::PassThrough<pcl::PointXYZ> pass;       //Create Filter

    pass.setInputCloud(input);
    pass.setFilterFieldName(filterField);
    pass.setFilterLimits(lowerLimit,upperLimit);
    if(negative){
        pass.setFilterLimitsNegative(true);
    }

    pass.filter(*output);   //Filter cloud based on above parameters

}

//Convert Down Method
void ConvertToPCL(sensor_msgs::PointCloud2ConstPtr input, PointCloud::Ptr output){

    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(*input, pcl_pc2);    //Converts from ROS Point Cloud 2 to PCL Point Cloud 2
    pcl::fromPCLPointCloud2(pcl_pc2,*output);      //Converts from PCL Point Cloud 2 to PCL Point Cloud

}

//Convert Up Method
void ConvertFromPCL(PointCloud::Ptr input, sensor_msgs::PointCloud2* output){

    pcl::PCLPointCloud2 pcl_pc2;
    pcl::toPCLPointCloud2(*input,pcl_pc2);      //Convert from PCL Point Cloud to PCL Point Cloud 2
    pcl_conversions::fromPCL(pcl_pc2,*output);  //Convert from PCL Point Cloud 2 to ROS Point Cloud 2

}

//Bounding Box Method(Center + Size)
void getBoundingBox(PointCloud::Ptr input,geometry_msgs::Pose* pose, geometry_msgs::Vector3* dimensions){

    Eigen::Vector4f min, max; //Vectors to hold max and min points
    pcl::getMinMax3D(*input,min,max); //Get max and min points
    
    //Determine the Position of the box based on the mid point of the min and max value
    pose->position.x = (max.x() + min.x())/2;
    pose->position.y = (max.y() + min.y())/2;
    pose->position.z = (max.z() + min.z())/2;

    //Determine the larger size of the bounding box 
    float xDiff = max.x() - min.x();
    float yDiff = max.y() - min.y();
    float xySize = 0;

    //If both sizes are smaller than the minimum set it to minimum
    if(xDiff< MINIMUMBOXSIZE && yDiff < MINIMUMBOXSIZE){
        xySize = MINIMUMBOXSIZE;
    //If both size are bigger than minimum set it to larger of two
    }else if(xDiff>MINIMUMBOXSIZE && yDiff>MINIMUMBOXSIZE){
        if (xDiff>yDiff){
            xySize = xDiff;
        }else{
            xySize = yDiff;
        }
    //If only x is bigger than minimum
    }else if(xDiff>MINIMUMBOXSIZE){
        xySize = xDiff;
    //All other conditions fail y must be biggest
    }else{
        xySize = yDiff;
    }

    //Assign bounding box size to message 
    dimensions->x =  xySize*BOUNDINGBOXSIZESCALER;
    dimensions->y = xySize*BOUNDINGBOXSIZESCALER;
    dimensions->z = BOUNDINGBOXHEIGHTSCALER;

}

//Covnert Bounding Box Styles
void coverttoPCLBox(Eigen::Vector4f& min, Eigen::Vector4f& max, geometry_msgs::Pose* pose, geometry_msgs::Vector3* dimensions){

    min[0] = pose->position.x - (dimensions->x/2);
    min[1] = pose->position.y - (dimensions->y/2);
    min[2] = pose->position.z - (dimensions->z/2);
    min[3] = 1.0;

    max[0] = pose->position.x + (dimensions->x/2);
    max[1] = pose->position.y + (dimensions->y/2);
    max[2] = pose->position.z + (dimensions->z/2);
    max[3] = 1.0;
    
}
//Convert Bounding Box to ROS Style
void coverttoROSBox(Eigen::Vector4f& min, Eigen::Vector4f& max,geometry_msgs::Pose* pose, geometry_msgs::Vector3* dimensions){

    //Determine the Position of the box based on the mid point of the min and max value
    pose->position.x = (max.x() + min.x())/2;
    pose->position.y = (max.y() + min.y())/2;
    pose->position.z = (max.z() + min.z())/2;

    //Determine the size of the box from difference between max and min value
    dimensions->x = (max.x() - min.x());
    dimensions->y = (max.y() - min.y());
    dimensions->z = (max.z() - min.z());

}
//Clustering Method

//Subscriber Callback
void PCLcallback(const sensor_msgs::PointCloud2ConstPtr& input){

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

    //Round Min and max to whole area in round numbers
    for(int i = 0; i < min.size(); i++){
        min[i] = floor(min[i]);
        max[i] = ceil(max[i]);
    }

	#ifdef ALLSTAGES
    	visualization_msgs::MarkerArray GPRmarkers; //Marker array message to show how cloud is split
    	int f =0; //Initialise marker id variable
	#endif

    PointCloud::Ptr gpr(new PointCloud); //Cloud to store new cloud with ground plane removed
    //Initialise vector for crop boxes and set constant value
    Eigen::Vector4f cbmin,cbmax;
    cbmin[2]=(float)min[2];
    cbmin[3]=1.0;
    cbmax[2]=(float)max[2];
    cbmax[3]=1.0f;
    
    for(float fx = min[0]; fx < max[0]; fx += GPRX){ //First in X axis
		//Set new bounds for x part of box
		cbmin[0]=(float)fx;
		cbmax[0]=(float)fx+GPRX;
		for(float fy = min[1]; fy < max[1]; fy += GPRY){ //Then in Y axis

			//Set new bound for y part of box
			cbmin[1]=(float)fy;  
			cbmax[1]=(float)fy+GPRY; 

			//Extract section from fov trimmed cloud and store it to cropped cloud
			pcl::CropBox<Point> adaptiveRemoval;
			adaptiveRemoval.setInputCloud(post);
			adaptiveRemoval.setMax(cbmax);
			adaptiveRemoval.setMin(cbmin);
			PointCloud::Ptr croppedCloud(new PointCloud);
			adaptiveRemoval.filter(*croppedCloud);

			//If the new cloud has no points in it then we do not need to remove ground plane (no plane exists)
			if(croppedCloud->size()>0){
				//Get the minimum and max points in the field (max is not used)
				Eigen::Vector4f gprmin, gprmax; 
				pcl::getMinMax3D(*croppedCloud,gprmin,gprmax);
				//Extract Points that lie above the lowest point + delta
				PointCloud::Ptr gprSect(new PointCloud);
				passThroughFilter(croppedCloud, "z", gprmin[2],gprmin[2]+GPRDELTA, gprSect, true);

				*gpr+=*gprSect; //Add filtered cloud to full ground plane removal cloud

			}	

			#ifdef ALLSTAGES
				//TODO Move to single method (Code dupe l300)
				visualization_msgs::Marker object_marker;
				object_marker.header.frame_id="velodyne";
				object_marker.header.stamp = ros::Time();
				object_marker.ns = "GPR";
				object_marker.id = f;
				object_marker.type = visualization_msgs::Marker::CUBE;
				coverttoROSBox(cbmin, cbmax, &object_marker.pose, &object_marker.scale);
				if (f%2 ==1){
					object_marker.color.b=1;
				}else{
					object_marker.color.r=1;
				}
				object_marker.color.a=0.3;
				GPRmarkers.markers.push_back(object_marker);

			#endif
			f++;
        }         
	}

	//Begin Clustering
    pcl::search::KdTree<Point>::Ptr tree (new pcl::search::KdTree<Point>);
    tree->setInputCloud(gpr);
    std::vector<pcl::PointIndices> cluster_indices;
    
    //Determine all clusters passed on the set parameters
	pcl::EuclideanClusterExtraction<Point> ec;
	ec.setClusterTolerance(CLUSTERTOLERANCE);
	ec.setMinClusterSize(MINCLUSTERSIZE);
	ec.setMaxClusterSize(MAXCLUSTERSIZE);
	ec.setSearchMethod(tree);
	ec.setInputCloud(gpr);
	ec.extract(cluster_indices);
    //Finished Clustering
    
	//TODO MAKE THIS IFDEF ABLE (only needed for display)
	int j = 0;
	visualization_msgs::MarkerArray markers;

    PointCloud::Ptr restoredCones(new PointCloud);
	for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
	{
		PointCloud::Ptr cloud_cluster (new PointCloud);
		for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
		cloud_cluster->points.push_back(gpr->points[*pit]); 
		cloud_cluster->width = cloud_cluster->points.size();
		cloud_cluster->height = 1;
		cloud_cluster->is_dense = true;
		//Create Point Cloud of each cluster as this is needed to determine min max

        //TODO Move to single method (Code dupe l230)
        //Create Marker Box for Cone
        visualization_msgs::Marker object_marker;
        object_marker.header.frame_id="velodyne";
        object_marker.header.stamp = ros::Time();
        object_marker.ns = "cluster";
        object_marker.id = j;
        object_marker.type = visualization_msgs::Marker::CUBE;
        getBoundingBox(cloud_cluster, &object_marker.pose, &object_marker.scale);
        object_marker.color.g=1;
        object_marker.color.a=0.3;
		markers.markers.push_back(object_marker);
        //Finished Creating Marker for Cone

        //Using Marker Variables heavily ties two parts together conside reformating in future

		/*
			Get min max pcl
			convert to ros box with scaler
			convert to pcl box points

			or

			get min max pcl
			scale by multiplying pcl points 
			do crop box
			covert to ros box

		*/

        //Begin Cone Restoration
        pcl::CropBox<Point> cb;
		cb.setInputCloud(pre);
        Eigen::Vector4f cbmin, cbmax; //Vectors to hold max and min points
        coverttoPCLBox(cbmin,cbmax, &object_marker.pose, &object_marker.scale);
		cb.setMax(cbmax);
		cb.setMin(cbmin);
		PointCloud::Ptr croppedCloud(new PointCloud);
		cb.filter(*croppedCloud);
        //Finished Cone Restoration

        *restoredCones+=*croppedCloud;

		//End of Statement
		j++;
	}

	//If defined will publish all points of pipeline
    #ifdef ALLSTAGES
        //Field of View Trimmed Cloud
        sensor_msgs::PointCloud2 fovTrimmed;
        ConvertFromPCL(post,&fovTrimmed);
        fovTrimmed.header.frame_id = "velodyne";
        fovTrimPub.publish(fovTrimmed);

        //Ground Plane Removal Cloud
        sensor_msgs::PointCloud2 gprRemoved;
        ConvertFromPCL(gpr,&gprRemoved);
        gprRemoved.header.frame_id = "velodyne";
        groundPlaneRemovalPub.publish(gprRemoved);

		GPRmarkerPub.publish(GPRmarkers); //Publish all the markers showing the different boxes for adaptive ground plane removal

		//Publisher marker Array (Bounding Boxes of cone candidates)
		ROS_INFO("Marker Array Size: %u", (int)markers.markers.size());
		markerPub.publish(markers);

    #endif

	//This is current end product and should be visualised at all times
	//Cone Restoration Cloud
	sensor_msgs::PointCloud2 restoredConesCloud;
	ConvertFromPCL(restoredCones,&restoredConesCloud);
	restoredConesCloud.header.frame_id = "velodyne";
	restoredPub.publish(restoredConesCloud);

}

//Main Method
int main(int argc, char **argv){

    ros::init(argc,argv, "PCL_Listener");

    ros::NodeHandle n; //Create Node

    //Subscribers
    ros::Subscriber sub = n.subscribe("velodyne_points", 1, PCLcallback); //Create Subscriber to velodyne

    //Publishers
	#ifdef ALLSTAGES
		fovTrimPub = n.advertise<PointCloud>("fov_trimmed_cloud",1);                    //Create Publisher for trimmed cloud
		groundPlaneRemovalPub = n.advertise<PointCloud>("gpr_cloud",1);                 //Create Publisher for GPR cloud
		GPRmarkerPub = n.advertise<visualization_msgs::MarkerArray>("gpr_markers",1); 	//Create Publisher for Marker array for ground plane removal areas
		markerPub = n.advertise<visualization_msgs::MarkerArray>("cone_markers",1);     //Create Publisher for Marker array for cone areas	
	#endif

	restoredPub = n.advertise<PointCloud>("restored_cones",1); 						//Create Publisher for restored cones cloud
    
	ros::spin();

    return(0);
}