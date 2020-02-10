#include "lidar_processing.hpp"

/**
 * @brief Initialise a LidarProcessing Object
 */
void LidarProcessing::initialise(){

    if(!initialised_){

        //lidarTopic = "/velodyne";
        // nh_.param("lidar",lidarTopic);
        lidarTopic = "/velodyne_points";
        lidar_sub_ = nh_.subscribe<sensor_msgs::PointCloud2>(lidarTopic, 1, boost::bind(&LidarProcessing::lidarCallback, this, _1));
        waitForSubscribers();

        //nh_.param("cone",coneTopic); 
        coneTopic = "/cones";
        cone_pub_ = nh_.advertise<perception_pkg::Cone>(coneTopic, 1);
        waitForPublishers();
        
/*         //FOV Trimming Parameters
        nh_.param("fov_trimming_lower_limit", fovLowerLimit);
        nh_.param("fov_trimming_upper_limit", fovUpperLimit);

        //Ground Plane Removal Parameters
        nh_.param("ground_plane_removal_x_step", gprXStep);
        nh_.param("ground_plane_removal_y_step", gprYStep);
        nh_.param("ground_plane_removal_delta", gprDelta);

        //Clustering Parameters
        nh_.param("euc_cluster_tolerance", eucMaximumDistance);
        nh_.param("euc_cluster_minimum_size", eucMinimumClusterSize);
        nh_.param("euc_cluster_maximum_size", eucMaximumClusterSize);

        //Cone Restoration Parameters
        nh_.param("cone_restore_box_size_scaler", boundingBoxSizeScaler);
        nh_.param("cone_restore_box_height_scaler", boundingBoxHeightScaler);
        nh_.param("cone_restore_box_minimum_size", minimumBoxSize);

        //Lidar Parameters
        nh_.param("lidar_vertical_resolution", lidarVerticalRes);
        nh_.param("lidar_horizontal_resolution", lidarHorizontalRes);

        //Cone Validity Check
        nh_.param("minimum_cloud_percentage", minimumCloudPercentage); */

        initialised_=true;
    } else{
        ROS_WARN("LidarProcessing object has already been initialised, skipping call");
    }
}

/**
 * @brief Convert ROS Style PointCloud2 to PCL PointCloud used by all pcl functions 
 * @param rosCloud PointCloud2 in ROS format
 * @param pclCloud PCL PointCloud for output
 */
void LidarProcessing::convertToPCL(sensor_msgs::PointCloud2 rosCloud, PointCloud::Ptr pclCloud){
    pcl::PCLPointCloud2 pclPC2;
    pcl_conversions::toPCL(rosCloud, pclPC2);    //Converts from ROS Point Cloud 2 to PCL Point Cloud 2
    pcl::fromPCLPointCloud2(pclPC2,*pclCloud);   //Converts from PCL Point Cloud 2 to PCL Point Cloud
}

/**
 * @brief Convert from PCL Style PointCloud to ROS PointCloud2 for communication between nodes
 * @param pclCloud PointCloud in PCL Format
 * @param rosCloud ROS PointCloud2 for output
 */
void LidarProcessing::convertToROS(PointCloud::Ptr pclCloud, sensor_msgs::PointCloud2Ptr rosCloud){
    pcl::PCLPointCloud2 pclPC2;
    pcl::toPCLPointCloud2(*pclCloud,pclPC2);      //Convert from PCL Point Cloud to PCL Point Cloud 2
    pcl_conversions::fromPCL(pclPC2,*rosCloud);  //Convert from PCL Point Cloud 2 to ROS Point Cloud 2
}

/**
 * @brief returns centre point of cloud
 * @param pclMinimum minimum point of cloud in PCL format 
 * @param pclMaximum maximum point of cloud in PCL format
 * @return centre point of cloud
 */
Eigen::Vector4f LidarProcessing::centrePoint(Eigen::Vector4f pclMinimum, Eigen::Vector4f pclMaximum){
    return (pclMaximum + pclMinimum)/2;
}

/**
 * @brief returns centre point of cloud
 * @param pclCloud pointcloud for which the centre will be found
 * @return centre point of cloud
 */
Eigen::Vector4f LidarProcessing::centrePoint(PointCloud::ConstPtr pclCloud){
    Eigen::Vector4f min, max; //Vectors to hold max and min points
    pcl::getMinMax3D(*pclCloud,min,max); //Get max and min points
    return LidarProcessing::centrePoint(min,max);
}

/**
 * @brief returns the overall dimensions of pointcloud
 * @param pclMinimum minimum point of cloud in PCL format 
 * @param pclMaximum maximum point of cloud in PCL format
 * @return size of cloud
 */
Eigen::Vector4f LidarProcessing::boxSize(Eigen::Vector4f pclMinimum, Eigen::Vector4f pclMaximum){
    return pclMaximum-pclMinimum;
}

/**
 * @brief returns the overall dimensions of pointcloud 
 * @param pclCloud cloud for which overall size will be found
 * @return size of cloud
 */
Eigen::Vector4f LidarProcessing::boxSize(PointCloud::ConstPtr pclCloud){
    Eigen::Vector4f min, max; //Vectors to hold max and min points
    pcl::getMinMax3D(*pclCloud,min,max); //Get max and min points
    return LidarProcessing::boxSize(min,max);
}

/**
 * @brief I dont like this
 */
void LidarProcessing::getBoundingBox(PointCloud::ConstPtr pclInputCloud, Eigen::Vector4f& pclMin, Eigen::Vector4f& pclMax){

    Eigen::Vector4f centre, size, newSize;
    size = boxSize(pclInputCloud);
    centre = centrePoint(pclInputCloud);

    double biggestAxis = std::max(size.x(),size.y());
    if(biggestAxis>minimumBoxSize){
        newSize.x() = biggestAxis*boundingBoxSizeScaler;
        newSize.y() = biggestAxis*boundingBoxSizeScaler;
    }else{
        newSize.x() = minimumBoxSize*boundingBoxSizeScaler;
        newSize.y() = minimumBoxSize*boundingBoxSizeScaler;
    }


    newSize.z() = boundingBoxHeightScaler;
    convertToPCLBox(newSize, centre, pclMin, pclMax);

}

/**
 * @brief or this
 */
void LidarProcessing::convertToPCLBox(Eigen::Vector4f size, Eigen::Vector4f centre, Eigen::Vector4f& pclMin, Eigen::Vector4f& pclMax){
    pclMin = centre - (size/2);
    pclMax = centre + (size/2);
}

/**
 * @brief and this one but it might not be needed
 */
void LidarProcessing::convertToROSBox(Eigen::Vector4f& pclMin, Eigen::Vector4f& pclMax, geometry_msgs::Pose* rosPose, geometry_msgs::Vector3* rosDimensions){
}

/**
 * @brief Filter to remove points within set bounds
 * @param pclInputCloud cloud to be filtered
 * @param pclOutputCloud cloud for filtered points to be returned to
 * @param filterfield field (Axis) for the cloud to be filtered on
 * @param lowerLimit value for lower bounds of filter
 * @param upperLimit value for upper bounds of filter
 * @param negate negate action of filter (points outwith bounds will be kept) 
 */
void LidarProcessing::passFilter(PointCloud::ConstPtr pclInputCloud, PointCloud::Ptr pclOutputCloud, std::string filterField, double lowerLimit, double upperLimit, bool negate){
    passThroughFilter.setInputCloud(pclInputCloud);
    passThroughFilter.setFilterFieldName(filterField);
    passThroughFilter.setFilterLimits(lowerLimit,upperLimit);
    if(negate){
        passThroughFilter.setFilterLimitsNegative(true);
    }

    passThroughFilter.filter(*pclOutputCloud);   //Filter cloud based on above parameters
}

/**
 * @brief Segment cloud into small sections and removed lower points
 * @param pclInputCloud cloud to be filtered
 * @param pclOutputCloud cloud for filtered points to be returned to
 * @param xStepsize size of sections in x direction
 * @param yStepsize size of sections in y direction
 * @param delta increase to minimum point from which point are removed
 */
void LidarProcessing::adaptiveGroundPlaneRemoval(PointCloud::ConstPtr pclInputCloud, PointCloud::Ptr pclOutputCloud, double xStepSize, double yStepSize, double delta){
    //Begins Ground Plane Removal
    Eigen::Vector4f min, max; //Vectors to hold max and min points
    pcl::getMinMax3D(*pclInputCloud,min,max);

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
    
	cropBoxFilter.setInputCloud(pclInputCloud);

    for(float fx = min.x(); fx < max.x(); fx += xStepSize){ //First in X axis
		//Set new bounds for x part of box
		cbMin.x()=fx;
		cbMax.x()=fx+xStepSize;
		for(float fy = min.y(); fy < max.y(); fy += yStepSize){ //Then in Y axis

			//Set new bound for y part of box
			cbMin.y()=fy;  
			cbMax.y()=fy+yStepSize; 

			//Extract section from fov trimmed cloud and store it to cropped cloud
			cropBoxFilter.setMax(cbMax);
			cropBoxFilter.setMin(cbMin);
			PointCloud::Ptr croppedCloud(new PointCloud);
			cropBoxFilter.filter(*croppedCloud);

			//If the new cloud has no points in it then we do not need to remove ground plane (no plane exists)
			if(croppedCloud->size()>0){

				//Get the minimum and max points in the field (max is not used)
				Eigen::Vector4f gprMin, gprMax; 
				pcl::getMinMax3D(*croppedCloud,gprMin,gprMax);

				//Extract Points that lie above the lowest point + delta
				PointCloud::Ptr gprSect(new PointCloud);
				LidarProcessing::passFilter(croppedCloud,gprSect, "z", gprMin.z(), gprMin.z()+gprDelta, true);

				*pclOutputCloud+=*gprSect; //Add filtered cloud to full ground plane removal cloud
			}	
			
        }         
	}//END OF GPR LOOP
}

/**
 * @brief method for extraction of cone candidates
 * @param pclInputCloud cloud to be processed
 * @param clusters vector of cone candidates
 * @param maximumSeperation maximum distance points can be seperatec before disregarded from cluster
 * @param minimumClusterSize minimum number of points that can make up a cluster
 * @param maximumClusterSize maximum number of points that can make up a cluster
 */
void LidarProcessing::eucCluster(PointCloud::ConstPtr pclInputCloud, std::vector<pcl::PointIndices>& clusters, double maximumSeperation, int minimumClusterSize, int maximumClusterSize){
    //Creare tree for search 
    pcl::search::KdTree<Point>::Ptr tree (new pcl::search::KdTree<Point>);
    tree->setInputCloud(pclInputCloud);

    //Determine all clusters passed on the set parameters
	
	eucClustering.setClusterTolerance(maximumSeperation);
	eucClustering.setMinClusterSize(minimumClusterSize);
	eucClustering.setMaxClusterSize(maximumClusterSize);
	eucClustering.setSearchMethod(tree);
	eucClustering.setInputCloud(pclInputCloud);
	eucClustering.extract(clusters);
}

/**
 * @brief extract cone candidate points from cloud 
 * @param pclInputCloud cloud that points are extracted from (Should be as small as possible)
 * @param pclPointClusters vector of clusters that are to be extracted
 * @return vector of all restored cones
 */
std::vector<PointCloud::Ptr> LidarProcessing::extractCandidates(PointCloud::ConstPtr pclInputCloud, std::vector<pcl::PointIndices>& pclPointClusters){
    std::vector<PointCloud::Ptr> cones;

    for (std::vector<pcl::PointIndices>::const_iterator it = pclPointClusters.begin(); it != pclPointClusters.end(); ++it){ //For every set of point indicies in clusters

        PointCloud::Ptr cloudCluster (new PointCloud); //Empty cloud for cone
        //Reconstruct point cloud from point indices
		for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit) 
		cloudCluster->points.push_back(pclInputCloud->points[*pit]); 
		cloudCluster->width = cloudCluster->points.size();
		cloudCluster->height = 1;
		cloudCluster->is_dense = true;
        cones.push_back(cloudCluster);
    }

    return cones;
}

/**
 * @brief restore cone candidates from unfiltered cloud
 * @param pclInputCloud cloud that cones will be restored from
 * @param coneCandidateClouds vector of all unrestored cones
 * @return vector of all restored cones
 */
std::vector<PointCloud::Ptr> LidarProcessing::restoreCandidates(PointCloud::ConstPtr pclInputCloud, std::vector<PointCloud::Ptr>& coneCandidateClouds){
    
    std::vector<PointCloud::Ptr> restoredCones;
	cropBoxFilter.setInputCloud(pclInputCloud);

    for (std::vector<PointCloud::Ptr>::const_iterator it = coneCandidateClouds.begin(); it !=coneCandidateClouds.end(); ++it){//Restore every cone in cone candidate list
        Eigen::Vector4f cbMin, cbMax; //Vectors to hold max and min points
        //Patch solution for tie between message and bounding box creation
        getBoundingBox(*it, cbMin, cbMax);

        //Begin Cone Restoration
		cropBoxFilter.setMax(cbMax);
		cropBoxFilter.setMin(cbMin);
		PointCloud::Ptr croppedCloud(new PointCloud);
		cropBoxFilter.filter(*croppedCloud);
        restoredCones.push_back(croppedCloud);
        //Finished Cone Restoration
    }

    return restoredCones;
}

/**
 * @brief Check cone is valid based on shape properties
 * @param pclInputCloud cone cloud that is to be checked for validity
 * @param lidarVerticalResolution the vertical resolution of the lidar (In Radians)
 * @param lidarHorizontalResolution the horizontal resolution of the lidar (In Radians)
 * @returns probability of cloud being cloud between 0.0 and 1.0 (Well at least it should)
 */
double LidarProcessing::coneValiditycheck(PointCloud::ConstPtr pclInputCloud, double  lidarVerticalResolution, double lidarHorizontalResolution){

    Eigen::Vector4f min, max, centre, size; //Vectors to hold max and min points centre point of box and size of box
    pcl::getMinMax3D(*pclInputCloud,min,max); //Get max and min points
    size = boxSize(min,max); //Get the size of the box X by Y by Z
    centre = centrePoint(min,max); //Get the centre point of the box
    float hc,wc,d; //Variables for storing needed values for cone check 

    hc = size.z(); //Heigh of the cone
    wc = sqrt( pow( size.x(), 2) + pow( size.y(), 2) ); //Length of Diagonal of base
    d = sqrt( pow( centre.x(), 2) + pow( centre.y(), 2) ); //Straigh line distance to cone from Lidar

 
    //Equation to determine number of point that should be in cone cloud
    float eD = 0.5*(hc/(2*d*atan2(lidarVerticalResolution,2)))*(wc/(2*d*atan2(lidarHorizontalResolution,2)));

    return roundf(pclInputCloud->size()/eD);
}

void LidarProcessing::findCones(){
    ROS_INFO("Attempting to Find Cones");
    ros::Time begin = ros::Time::now();

    PointCloud::Ptr rawCloud(new PointCloud);
    convertToPCL(global_cloud_, rawCloud);

    PointCloud::Ptr fovTrimCloud(new PointCloud);
    passFilter(rawCloud, fovTrimCloud, "x", fovLowerLimit, fovUpperLimit, false);

    PointCloud::Ptr groundPlaneRemovalCloud(new PointCloud);
    adaptiveGroundPlaneRemoval(fovTrimCloud, groundPlaneRemovalCloud, gprXStep, gprYStep, gprDelta);

    std::vector<pcl::PointIndices> clusterIndices;
    eucCluster(groundPlaneRemovalCloud, clusterIndices, eucMaximumDistance, eucMinimumClusterSize, eucMaximumClusterSize);

    std::vector<PointCloud::Ptr> cones = extractCandidates(groundPlaneRemovalCloud, clusterIndices);
    ROS_INFO("Cone Candidates %d", cones.size());

    std::vector<PointCloud::Ptr> restoredCones = restoreCandidates(rawCloud, cones);
    ROS_INFO("Cones Restored %d" ,restoredCones.size());


}
/**
 * @brief Waits for publiser to be ready to publish messages
 */
void LidarProcessing::waitForPublishers(){
    ros::Rate rate(20);
    while(ros::ok()){
        if(cone_pub_.getNumSubscribers()>0){
            cone_publisher_ready_ = true;
            ROS_WARN("Publisher is now ready");
            break;
        }
    }
}

/**
 * @brief Waits for subscribers to start receiving messages 
 * @todo Implement Correct method
 */
void LidarProcessing::waitForSubscribers(){
    
    lidar_subscriber_active_ = true;
    while(!lidar_subscriber_active_){
        ROS_WARN("Lidar has begun publishing data");
    }
}

/**
 * @brief Callback for recieving lidar data
 * @param msg a lidar message
 */
void LidarProcessing::lidarCallback(const sensor_msgs::PointCloud2ConstPtr& msg){
    ROS_INFO("LIDAR CALLBACK TRIGGERED");
    global_cloud_.header = msg->header;
    global_cloud_.height = msg->height;
    global_cloud_.width = msg->width;
    global_cloud_.fields = msg->fields;
    global_cloud_.is_bigendian = msg->is_bigendian;
    global_cloud_.point_step = msg->point_step;
    global_cloud_.row_step = msg->row_step;
    global_cloud_.data = msg->data;
    global_cloud_.is_dense = msg->is_dense;

}


int main(int argc, char** argv){
    ros::init(argc,argv, "Lidar_Processing_Node");
    ros::NodeHandle nodeHandle;
    LidarProcessing lp(&nodeHandle);

    lp.initialise();
    //Add do part here
    lp.findCones();
    ros::spin();
    return 0;

}