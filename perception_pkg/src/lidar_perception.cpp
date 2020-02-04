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
#define MINCLUSTERSIZE 3
#define MAXCLUSTERSIZE 20

//Bounding Box Parameters
#define BOUNDINGBOXSIZESCALER 3
#define BOUNDINGBOXHEIGHTSCALER 1
#define MINIMUMBOXSIZE 0.08

//Sensor Parameters
#define VERTICALRES 0.0349066 //2 //Degrees 
#define HORIZONTALRES 0.00349066//0.2 //Degrees
//Debug Parameters
#define ALLSTAGES

//Global Publisher creation
#ifdef ALLSTAGES
	ros::Publisher fovTrimPub;
	ros::Publisher groundPlaneRemovalPub;
	ros::Publisher markerPub;
	//ros::Publisher GPRmarkerPub;
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

    pcl::PCLPointCloud2 pclPC2;
    pcl_conversions::toPCL(*input, pclPC2);    //Converts from ROS Point Cloud 2 to PCL Point Cloud 2
    pcl::fromPCLPointCloud2(pclPC2,*output);      //Converts from PCL Point Cloud 2 to PCL Point Cloud

}

//Convert Up Method
void ConvertFromPCL(PointCloud::Ptr input, sensor_msgs::PointCloud2* output){

    pcl::PCLPointCloud2 pclPC2;
    pcl::toPCLPointCloud2(*input,pclPC2);      //Convert from PCL Point Cloud to PCL Point Cloud 2
    pcl_conversions::fromPCL(pclPC2,*output);  //Convert from PCL Point Cloud 2 to ROS Point Cloud 2

}

//Find centre of min and max
Eigen::Vector4f centrePoint(Eigen::Vector4f min, Eigen::Vector4f max ){
    return (max + min)/2;
}

//Get the Size of a box based on min and max
Eigen::Vector4f boxSize(Eigen::Vector4f min, Eigen::Vector4f max){
    return max - min;
}

//Bounding Box Method(Center + Size) TODO REFACTOR
void getBoundingBox(PointCloud::Ptr input,geometry_msgs::Pose* pose, geometry_msgs::Vector3* dimensions){

    Eigen::Vector4f min, max, centre, size; //Vectors to hold max and min points
    pcl::getMinMax3D(*input,min,max); //Get max and min points
    centre = centrePoint(min,max);
    //Get the centre point of the box
    pose->position.x = centre.x();
    pose->position.y = centre.y();  
    pose->position.z = centre.z();

    //Determine the larger size of the bounding box 
    size = boxSize(min,max);
    float xySize = 0;

    //If both sizes are smaller than the minimum set it to minimum
    if(size.x()< MINIMUMBOXSIZE && size.y() < MINIMUMBOXSIZE){
        xySize = MINIMUMBOXSIZE;
    //If both size are bigger than minimum set it to larger of two
    }else if(size.x()>MINIMUMBOXSIZE && size.y()>MINIMUMBOXSIZE){
        if (size.x()>size.y()){
            xySize = size.x();
        }else{
            xySize = size.y();
        }
    //If only x is bigger than minimum
    }else if(size.x()>MINIMUMBOXSIZE){
        xySize = size.x();
    //All other conditions fail y must be biggest
    }else{
        xySize = size.y();
    }

    //Assign bounding box size to message 
    dimensions->x =  xySize*BOUNDINGBOXSIZESCALER;
    dimensions->y = xySize*BOUNDINGBOXSIZESCALER;
    dimensions->z = BOUNDINGBOXHEIGHTSCALER;

}

//Covnert Bounding Box Styles
void coverttoPCLBox(Eigen::Vector4f& min, Eigen::Vector4f& max, geometry_msgs::Pose* pose, geometry_msgs::Vector3* dimensions){

    //Take Pose and Dimensions and convert to minimum point of pcl "The bottom left corner"
    min[0] = pose->position.x - (dimensions->x/2);
    min[1] = pose->position.y - (dimensions->y/2);
    min[2] = pose->position.z - (dimensions->z/2);
    min[3] = 1.0;

    //Take Pose and Dimensions and convert to maximum point of pcl "The top right corner"
    max[0] = pose->position.x + (dimensions->x/2);
    max[1] = pose->position.y + (dimensions->y/2);
    max[2] = pose->position.z + (dimensions->z/2);
    max[3] = 1.0;
    
}

//Convert Bounding Box to ROS Style
void coverttoROSBox(Eigen::Vector4f& min, Eigen::Vector4f& max,geometry_msgs::Pose* pose, geometry_msgs::Vector3* dimensions){

    //Determine the Position of the box based on the mid point of the min and max value
    Eigen::Vector4f centre, size;
    centre = centrePoint(min,max);

    //Store centre point in ROS Pose data type
    pose->position.x = centre.x();
    pose->position.y = centre.y();  
    pose->position.z = centre.z();

    //Determine the size of the box from difference between max and min value
    size = boxSize(min,max);

    //Store box size in ROS Dimension data type
    dimensions->x = size.x();
    dimensions->y = size.y();
    dimensions->z = size.z();

}

void adaptiveGroundPlaneRemoval(PointCloud::Ptr input, PointCloud::Ptr output, float xStepSize, float yStepSize, float removalDelta){
    //Begins Ground Plane Removal
    Eigen::Vector4f min, max; //Vectors to hold max and min points
    pcl::getMinMax3D(*input,min,max);

    //Round Min and max to whole area in round numbers
    for(int i = 0; i < min.size(); i++){
        min[i] = floor(min[i]);
        max[i] = ceil(max[i]);
    }

    //Initialise vector for crop boxes and set constant value
    Eigen::Vector4f cbMin,cbMax;
    cbMin.z()=min.z();
    cbMin.w()=1.0;
    cbMax.z()=max.z();
    cbMax.w()=1.0;
    pcl::CropBox<Point> adaptiveRemoval;
	adaptiveRemoval.setInputCloud(input);

    for(float fx = min.x(); fx < max.x(); fx += xStepSize){ //First in X axis
		//Set new bounds for x part of box
		cbMin.x()=fx;
		cbMax.x()=fx+xStepSize;
		for(float fy = min.y(); fy < max.y(); fy += yStepSize){ //Then in Y axis

			//Set new bound for y part of box
			cbMin.y()=fy;  
			cbMax.y()=fy+yStepSize; 

			//Extract section from fov trimmed cloud and store it to cropped cloud
			adaptiveRemoval.setMax(cbMax);
			adaptiveRemoval.setMin(cbMin);
			PointCloud::Ptr croppedCloud(new PointCloud);
			adaptiveRemoval.filter(*croppedCloud);

			//If the new cloud has no points in it then we do not need to remove ground plane (no plane exists)
			if(croppedCloud->size()>0){

				//Get the minimum and max points in the field (max is not used)
				Eigen::Vector4f gprMin, gprMax; 
				pcl::getMinMax3D(*croppedCloud,gprMin,gprMax);

				//Extract Points that lie above the lowest point + delta
				PointCloud::Ptr gprSect(new PointCloud);
				passThroughFilter(croppedCloud, "z", gprMin.z(),gprMin.z()+removalDelta, gprSect, true);

				*output+=*gprSect; //Add filtered cloud to full ground plane removal cloud
			}	
			
        }         
	}//END OF GPR LOOP
}

//Clustering Method
void eucCluster(PointCloud::Ptr input, std::vector<pcl::PointIndices>& clusters, float clusterTolerance, int minimumClusterSize, int maximumClusterSize){
    //Creare tree for search 
    pcl::search::KdTree<Point>::Ptr tree (new pcl::search::KdTree<Point>);
    tree->setInputCloud(input);

    //Determine all clusters passed on the set parameters
	pcl::EuclideanClusterExtraction<Point> ec;
	ec.setClusterTolerance(clusterTolerance);
	ec.setMinClusterSize(minimumClusterSize);
	ec.setMaxClusterSize(maximumClusterSize);
	ec.setSearchMethod(tree);
	ec.setInputCloud(input);
	ec.extract(clusters);

}

//Based on the size of the cloud determine the probability of it being a cone based on the number of points
float coneCheck(PointCloud::Ptr input, float verticalResolution, float horizontalResolution){

    Eigen::Vector4f min, max, centre, size; //Vectors to hold max and min points centre point of box and size of box
    pcl::getMinMax3D(*input,min,max); //Get max and min points
    size = boxSize(min,max); //Get the size of the box X by Y by Z
    centre = centrePoint(min,max); //Get the centre point of the box
    float hc,wc,d; //Variables for storing needed values for cone check 

    hc = size.z(); //Heigh of the cone
    wc = sqrt( pow( size.x(), 2) + pow( size.y(), 2) ); //Length of Diagonal of base
    d = sqrt( pow( centre.x(), 2) + pow( centre.y(), 2) ); //Straigh line distance to cone from Lidar

 
    //Equation to determine number of point that should be in cone cloud
    float eD = 0.5*(hc/(2*d*atan2(verticalResolution,2)))*(wc/(2*d*atan2(horizontalResolution,2)));
    /*
        ROS_WARN("hc %f", hc);
        ROS_WARN("wc %f", wc);
        ROS_WARN("d %f", d);
        ROS_WARN("eD Value %d ",eD);
        ROS_WARN("CLOUD SIZE %d", input->size());
    */
    return roundf(input->size()/eD);
}

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

    //Begin Ground Plane Removal
    PointCloud::Ptr gpr (new PointCloud);
    adaptiveGroundPlaneRemoval(post, gpr, GPRX, GPRY, GPRDELTA);
    //Finished Ground Plane Removal

    //NOTE GPR MARKERS HAVE BEEN REMOVED

    //Begin clustering
    std::vector<pcl::PointIndices> clusterIndices;
    eucCluster(gpr, clusterIndices, CLUSTERTOLERANCE, MINCLUSTERSIZE, MAXCLUSTERSIZE);
    //End clustering
    
    #ifdef ALLSTAGES
	    int j = 0;
	    visualization_msgs::MarkerArray markers;
    #endif

    pcl::CropBox<Point> cb;
	cb.setInputCloud(pre);
    PointCloud::Ptr restoredCones(new PointCloud);
	for (std::vector<pcl::PointIndices>::const_iterator it = clusterIndices.begin (); it != clusterIndices.end (); ++it)
	{
		//Covert cluster indicies to point cloud to allow filtering
        PointCloud::Ptr cloudCluster (new PointCloud);
		for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
		cloudCluster->points.push_back(gpr->points[*pit]); 
		cloudCluster->width = cloudCluster->points.size();
		cloudCluster->height = 1;
		cloudCluster->is_dense = true;

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
        //Patch solution for tie between message and bounding box creation
        geometry_msgs::Pose centrePoint;
        geometry_msgs::Vector3 boxSize;
        getBoundingBox(cloudCluster, &centrePoint, &boxSize);

        //Begin Cone Restoration
        Eigen::Vector4f cbMin, cbMax; //Vectors to hold max and min points
        coverttoPCLBox(cbMin,cbMax, &centrePoint, &boxSize);
		cb.setMax(cbMax);
		cb.setMin(cbMin);
		PointCloud::Ptr croppedCloud(new PointCloud);
		cb.filter(*croppedCloud);
        //Finished Cone Restoration

        float prob = coneCheck(croppedCloud, VERTICALRES, HORIZONTALRES);
        ROS_INFO("PROBABILITY OF CONE %f", prob);

        *restoredCones+=*croppedCloud; //Add cropped cloud to restored cone full cloud (this isnt needed, only for visualisation)		

        #ifdef ALLSTAGES
            //TODO Move to single method (Code dupe l230)
            //Create Marker Box for Cone
            visualization_msgs::Marker objectMarker;
            objectMarker.header.frame_id="velodyne";
            objectMarker.header.stamp = ros::Time();
            objectMarker.ns = "cluster";
            objectMarker.id = j;
            objectMarker.type = visualization_msgs::Marker::CUBE;
            objectMarker.color.g=1;
            objectMarker.color.a=0.3;
            objectMarker.pose = centrePoint;
            objectMarker.scale = boxSize;
            markers.markers.push_back(objectMarker);
            j++;
        #endif
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

		//GPRmarkerPub.publish(GPRmarkers); //Publish all the markers showing the different boxes for adaptive ground plane removal

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
		//GPRmarkerPub = n.advertise<visualization_msgs::MarkerArray>("gpr_markers",1); 	//Create Publisher for Marker array for ground plane removal areas
		markerPub = n.advertise<visualization_msgs::MarkerArray>("cone_markers",1);     //Create Publisher for Marker array for cone areas	
	#endif

	restoredPub = n.advertise<PointCloud>("restored_cones",1); 						//Create Publisher for restored cones cloud
    
	ros::spin();

    return(0);
}