#include "lidar_processing.hpp"

/**
 * @brief Initialise a LidarProcessing Object
 */
void LidarProcessing::initialise(){

    if(!initialised_){

        nh_.param("lidar",lidarTopic); //Attempt to read lidar topic parameter
        //If unable to read in params from launch file then default to inbuilt ones
        if(lidarTopic ==""){
            ROS_ERROR("PARAMETERS FAIL TO READ, DEFAULTING TO INBUILT");
            //call some function
            defaultParams();
        }else{
            defaultLaunchParams();
        }

        //Setup Subscriber
        lidar_sub_ = nh_.subscribe<sensor_msgs::PointCloud2>(lidarTopic, 1, boost::bind(&LidarProcessing::lidarCallback, this, _1));
        waitForSubscribers();

        //Setup Publishers
        cone_pub_ = nh_.advertise<perception_pkg::Cone>(coneTopic, 1);
        if(forceSubs){
            waitForPublishers();
        }
        

        psuedo_scan_pub_ = nh_.advertise<sensor_msgs::LaserScan>("/laserscan",50);

        //Processing Pipeline Stages, used for debug
        if(processingSteps){
            fov_trim_pub_ = nh_.advertise<PointCloud>("lidar_debug/fov_trimmed_cloud",1);
		    ground_plane_removal_pub_ = nh_.advertise<PointCloud>("lidar_debug/gpr_cloud",1);
            restored_pub_ = nh_.advertise<PointCloud>("lidar_debug/restored_cones",1);
            cone_marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("lidar_debug/cone_markers",1);
            single_cone_pub_=nh_.advertise<PointCloud>("lidar_debug/single_cones",1);
            yellow_cone_cloud_pub_=nh_.advertise<PointCloud>("lidar_debug/yellow_cones",1);
            blue_cone_cloud_pub_=nh_.advertise<PointCloud>("lidar_debug/blue_cones",1);
        }

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
void LidarProcessing::convertToPCL(sensor_msgs::PointCloud2ConstPtr rosCloud, PointCloud::Ptr pclCloud){
    
    pcl::PCLPointCloud2 pclPC2;
    pcl_conversions::toPCL(*rosCloud, pclPC2);    //Converts from ROS Point Cloud 2 to PCL Point Cloud 2
    pcl::fromPCLPointCloud2(pclPC2,*pclCloud);   //Converts from PCL Point Cloud 2 to PCL Point Cloud
}

/**
 * @brief Convert from PCL Style PointCloud to ROS PointCloud2 for communication between nodes
 * @param pclCloud PointCloud in PCL Format
 * @param rosCloud ROS PointCloud2 for output
 */
void LidarProcessing::convertToROS(PointCloud::Ptr pclCloud, sensor_msgs::PointCloud2* rosCloud){
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
    }else{
        passThroughFilter.setFilterLimitsNegative(false);
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

    if(pclInputCloud->size() == 0){
        ROS_ERROR("Ma point clouds empty, you sure your on track?");

    }else{
        tree->setInputCloud(pclInputCloud);
    }
    

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

/**
 * @brief determine colour of cone 
 * @param pclInputCloud cone cloud that is to be checked for colour
 */
std::string LidarProcessing::coneColourcheck(PointCloud::ConstPtr pclInputCloud){
    
    /* //Begin implementation of new method will need to add pass in of band step
        Eigen::Vector4f min,max;
        getBoundingBox(pclInputCloud, min,max);
        float bands = coneColourBands; //Temp solution until call is changed
        
        float zDiff = max.z() -min.z(); //Total height of the cone cloud
        size_t segSize;
        for(float i = 0; i<bands; i++ ){
            
            float m = i/bands;
            float zBandStart = min.z() + (m*zDiff);

            float n = (i+1.0)/bands;
            float zBandEnd = min.z() + (n*zDiff);
            //ROS_INFO("M %f N %f", m,n);
            //ROS_INFO("Band Start %f , Band End %f ", zBandStart, zBandEnd);
            PointCloud::Ptr coneBand(new PointCloud);
            passFilter(pclInputCloud,coneBand,"z",zBandStart,zBandEnd,false);

            //Only begin next stage if band has points in it
            if(coneBand->points.size()){
                ROS_INFO("BAND %1f size %d", i+1, coneBand->points.size());
                segSize += coneBand->points.size();  
                float av =0;  
                for(int j =0; j<coneBand->points.size(); j++){
                    //ROS_INFO("Intensity value of point %d is %f",j, );
                    av +=(float)coneBand->points[i].intensity;
                    //std::cout<<"av "<<av<<" Current "<< coneBand->points[i].intensity << "\n";
                }
                //std::cout<<"\n";
                av = av/coneBand->points.size();
                ROS_INFO("Average Value %f", av);

            }
            
        }
        //Warn if points have been duplicated during splitting
        if(segSize>pclInputCloud->points.size() || segSize<pclInputCloud->points.size()){
            ROS_WARN("Points have been lost or duplicated: Original Size %d Segmented Size %d", pclInputCloud->size(), segSize);     
        } 
    */

    //Old method of determining colour
    Eigen::Vector4f centre = centrePoint(pclInputCloud);
    if(centre.y()>0){
        return "blue";
    }else{
        return "yellow";
    }
}

/**
 * @brief create message for containing cone information
 * @param pclInputCloud cone cloud that represents cone
 * @param coneColour colour of cone
 */
perception_pkg::Cone LidarProcessing::createConeMessage(PointCloud::ConstPtr pclInputCloud, std::string coneColour){
    perception_pkg::Cone cone;
    //Header
    cone.header.stamp = ros::Time::now();
    cone.header.frame_id = "Cone";
    //Child Frame ID
    cone.child_frame_id = "velodyne";

    //Colour
    cone.colour = coneColour;

    Eigen::Vector4f centre = centrePoint(pclInputCloud);

    cone.location.x = centre.x();
    cone.location.y = centre.y();
    cone.location.z = 0;
    return cone;
}

/**
 * @brief take an input cloud and find all the cones within it
 */
void LidarProcessing::findCones(){
    /*     
        ROS_INFO("Attempting to Find Cones");
        ros::Time begin = ros::Time::now();

        // PointCloud::Ptr rawCloud(new PointCloud);
        // convertToPCL(global_cloud_, rawCloud);

        PointCloud::Ptr fovTrimCloud(new PointCloud);
        passFilter(global_cloud_, fovTrimCloud, "x", fovLowerLimit, fovUpperLimit, false);

        PointCloud::Ptr groundPlaneRemovalCloud(new PointCloud);
        adaptiveGroundPlaneRemoval(fovTrimCloud, groundPlaneRemovalCloud, gprXStep, gprYStep, gprDelta);

        std::vector<pcl::PointIndices> clusterIndices;
        eucCluster(groundPlaneRemovalCloud, clusterIndices, eucMaximumDistance, eucMinimumClusterSize, eucMaximumClusterSize);

        std::vector<PointCloud::Ptr> cones = extractCandidates(groundPlaneRemovalCloud, clusterIndices);
        ROS_INFO("Cone Candidates %d", cones.size());

        std::vector<PointCloud::Ptr> restoredCones = restoreCandidates(global_cloud_, cones);
        ROS_INFO("Cones Restored %d" ,restoredCones.size()); 
    */


}

/**
 * @brief Fallback method for if param read in fails
 */
void LidarProcessing::defaultParams(){
    lidarTopic = "/velodyne_points";
    coneTopic = "/cones";

    fovLowerLimit = 0.0;
    fovUpperLimit = 5.0;

    gprXStep = 3.0;
    gprYStep = 1.0;
    gprDelta = 0.15;

    eucMaximumDistance = 0.3;
    eucMinimumClusterSize = 1;
    eucMaximumClusterSize = 20;

    boundingBoxSizeScaler = 3;
    boundingBoxHeightScaler = 1;
    minimumBoxSize = 0.08;

    lidarVerticalRes = 0.0349066;
    lidarHorizontalRes = 0.00349066;

    minimumCloudPercentage = 0.5;

    coneColourBands = 10;

    processingSteps = 1;
    forceSubs= 0;

}

/**
 * @brief Read Remaining Paramaters from Launch file
 */
void LidarProcessing::defaultLaunchParams(){

    nh_.param("cone",coneTopic); 
    //FOV Trimming Parameters
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
    nh_.param("minimum_cloud_percentage", minimumCloudPercentage); 
    nh_.param("processing_pipeline_stages", processingSteps);
}

/**
 * @brief Waits for publiser to be ready to publish messages
 */
void LidarProcessing::waitForPublishers(){
    ros::Rate rate(20);
    int update = 0;
    while(ros::ok()){
        if(cone_pub_.getNumSubscribers()>0){
            cone_publisher_ready_ = true;
            ROS_WARN("Publisher is now ready");
            break;
        }else{
            update++;
            if(update%1000000 == 0){
                ROS_WARN("No Subscribers to Cones");
            }
            
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
 * @brief Outputs the 3 main stages of the lidar pipeline to 3 seperate ros topics
 * @param trimmedCloud pointcloud from the fov trimming stage 
 * @param gprCloud pointcloud from ground plane removal stage
 * @param restoredCones vector of all restored cones
 */
void LidarProcessing::outputDebugClouds(PointCloud::Ptr trimmedCloud, PointCloud::Ptr gprCloud, std::vector<PointCloud::Ptr> restoredCones, std::vector<PointCloud::Ptr> yellowCones, std::vector<PointCloud::Ptr> blueCones){
    sensor_msgs::PointCloud2 fovTrimmed, gpr, restored, blueConesMessage, yellowConesMessage;
    convertToROS(trimmedCloud,&fovTrimmed);
    fovTrimmed.header.frame_id = "velodyne";
    fov_trim_pub_.publish(fovTrimmed);

    convertToROS(gprCloud,&gpr);
    gpr.header.frame_id = "velodyne";
    ground_plane_removal_pub_.publish(gpr);

    visualization_msgs::MarkerArray cones;
    PointCloud::Ptr restoredConeCloud(new PointCloud);
    for(int i =0; i<restoredCones.size(); i++){
        sensor_msgs::PointCloud2 cone; 
        convertToROS(restoredCones[i],&cone);
        cone.header.frame_id = "velodyne";
        single_cone_pub_.publish(cone);

        *restoredConeCloud += *restoredCones[i];
        visualization_msgs::Marker mark;

        mark.header.frame_id = "velodyne";
        mark.header.stamp = ros::Time::now();
        mark.ns = "Cones";
        mark.id = i;
        mark.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        mark.action = visualization_msgs::Marker::ADD;

        Eigen::Vector4f centre, min,max;
        centre = centrePoint(restoredCones[i]);
        getBoundingBox(restoredCones[i],min,max);

        mark.pose.position.x = centre.x();
        mark.pose.position.y = centre.y();
        mark.pose.position.z = (max.z() +0.1);
        mark.scale.x = 0.1;
        mark.scale.y = 0.1;
        mark.scale.z = 0.2;
        mark.color.a = 1.0;
        mark.color.g = 1.0;

        std::ostringstream stm;
        stm<<i;
        std::string s = "Cone " + stm.str(); 
        mark.text = s;
        cones.markers.push_back(mark);

    }
    cone_marker_pub_.publish(cones);

    convertToROS(restoredConeCloud,&restored);
    restored.header.frame_id = "velodyne";
    restored_pub_.publish(restored);

    PointCloud::Ptr yellowConeCloud(new PointCloud);
    for(int i =0; i<yellowCones.size(); i++){
        *yellowConeCloud+=*yellowCones[i];
    }
    convertToROS(yellowConeCloud,&yellowConesMessage);
    yellowConesMessage.header.frame_id="velodyne";
    yellow_cone_cloud_pub_.publish(yellowConesMessage);

    PointCloud::Ptr blueConeCloud(new PointCloud);
    for(int i =0; i<blueCones.size(); i++){
        *blueConeCloud+=*blueCones[i];
    }
    convertToROS(blueConeCloud,&blueConesMessage);
    blueConesMessage.header.frame_id="velodyne";
    blue_cone_cloud_pub_.publish(blueConesMessage);


    
}

/**
 * @brief creates a fake laser scan message for use in gmapping tests
 * @param coneClouds cones found during process
 */
sensor_msgs::LaserScan LidarProcessing::createDummyScan(std::vector<PointCloud::Ptr> coneClouds){
    sensor_msgs::LaserScan scan; //Empty Scan Message
    int num_readings = 720; //Number of fake readings
    int laser_frequency = 40; //Fake laser frequencys
    //Values assigened to scan message
    scan.header.stamp = ros::Time::now();
    scan.header.frame_id = "velodyne";
    scan.angle_min = -1.570796;
    scan.angle_max = 1.570796;
    scan.angle_increment = 3.14/num_readings;
    scan.time_increment = (1/laser_frequency)/num_readings;
    scan.range_min = 0.1;
    scan.range_max = 10.0;
    scan.scan_time = 1/laser_frequency;
    //Arrays for faked range and intensity values
    double ranges[num_readings];
    double intensities[num_readings];

    scan.ranges.resize(num_readings);
    scan.intensities.resize(num_readings);

    int inflationSize = 5; //Number of points added either side of the cone centre point

    for(int i = 0; i<num_readings; i++){
        scan.ranges[i]= std::numeric_limits<float>::infinity();
        scan.intensities[i]= 0;  
    }

    for(int i = 0; i<coneClouds.size(); i++){
        Eigen::Vector4f centre =centrePoint(coneClouds[i]);
        double angle = atan2(centre.y(),centre.x());

        int index = (angle-scan.angle_min)/scan.angle_increment; //Centre Point of cone in range of laser scan
        float distance = sqrtf(pow(centre.x(),2) + pow(centre.y(),2)); //Straigh line distance of cone

        for(int j = - inflationSize; j<inflationSize; j++){
            int pos = index +j;
            if(pos>=0 && pos<num_readings){
                scan.ranges[index+j] = distance;
                scan.intensities[index+j] = 0;
            }else{
                ROS_WARN("Cone out of bounds of laser scan, ignoreing point");
            }
            
        }
 
    }
    
    return scan;
}
/**
 * @brief Callback for recieving lidar data
 * @param msg a lidar message
 */
void LidarProcessing::lidarCallback(const sensor_msgs::PointCloud2ConstPtr& msg){

    ROS_WARN("Attempting to Find Cones");
    ros::Time begin = ros::Time::now();

    PointCloud::Ptr rawCloud(new PointCloud);
    convertToPCL(msg, rawCloud);

    PointCloud::Ptr fovTrimCloud(new PointCloud);
    passFilter(rawCloud, fovTrimCloud, "x", fovLowerLimit, fovUpperLimit, false);

    PointCloud::Ptr groundPlaneRemovalCloud(new PointCloud);
    adaptiveGroundPlaneRemoval(fovTrimCloud, groundPlaneRemovalCloud, gprXStep, gprYStep, gprDelta);

    std::vector<pcl::PointIndices> clusterIndices;
    eucCluster(groundPlaneRemovalCloud, clusterIndices, eucMaximumDistance, eucMinimumClusterSize, eucMaximumClusterSize);

    std::vector<PointCloud::Ptr> cones = extractCandidates(groundPlaneRemovalCloud, clusterIndices);

    std::vector<PointCloud::Ptr> restoredCones = restoreCandidates(rawCloud, cones);
    ROS_INFO("Cones Found: %d" , (int)restoredCones.size());
    std::vector<PointCloud::Ptr> yellow;
    std::vector<PointCloud::Ptr> blue;
    for(int i =0; i<restoredCones.size(); i++){
        double validity = coneValiditycheck(restoredCones[i],lidarVerticalRes,lidarHorizontalRes);
        Eigen::Vector4f centre= centrePoint(restoredCones[i]);
        ROS_INFO("Cone %d Pos X: %f Y: %f Z: %f Validity: %d",i,centre.x(),centre.y(),centre.z(), (int)validity);


        if(validity>=minimumCloudPercentage){
            std::string colour = coneColourcheck(restoredCones[i]);

            if(colour=="yellow"){
                yellow.push_back(restoredCones[i]);
            }else if(colour=="blue"){
                blue.push_back(restoredCones[i]);
            }

            cone_pub_.publish(createConeMessage(restoredCones[i],colour));
        }
    }

    if(processingSteps){
        outputDebugClouds(fovTrimCloud,groundPlaneRemovalCloud,restoredCones, yellow, blue);
    }
    psuedo_scan_pub_.publish(createDummyScan(restoredCones));
}


int main(int argc, char** argv){
    ros::init(argc,argv, "Lidar_Processing_Node");
    ros::NodeHandle nodeHandle;
    LidarProcessing lp(&nodeHandle);
    
    lp.initialise();
    ros::spin();
    return 0;

}