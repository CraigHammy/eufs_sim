#ifndef LIDAR_PROCESSING_ROS_HEADER //Header Guard
#define LIDAR_PROCESSING_ROS_HEADER

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
#include <perception_pkg/Cone.h>
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

class LidarProcessing{

    public:
        /**
         * @brief Constructer for LidarProcessing class
         */
        LidarProcessing(ros::NodeHandle* nh) : nh_(*nh), initialised_(false), lidar_subscriber_active_(false), cone_publisher_ready_(false), count_(0){}//Constructer 

        /**
         * @brief Initialise a LidarProcessing Object
         */
        void initialise();

        /**
         * @brief Convert ROS Style PointCloud2 to PCL PointCloud used by all pcl functions 
         * @param rosCloud PointCloud2 in ROS format
         * @param pclCloud PCL PointCloud for output
         */
        void convertToPCL(sensor_msgs::PointCloud2 rosCloud, PointCloud::Ptr pclCloud);

        /**
         * @brief Convert from PCL Style PointCloud to ROS PointCloud2 for communication between nodes
         * @param pclCloud PointCloud in PCL Format
         * @param rosCloud ROS PointCloud2 for output
         */
        void convertToROS(PointCloud::Ptr pclCloud, sensor_msgs::PointCloud2Ptr rosCloud);
        
        /**
         * @brief returns centre point of cloud
         * @param pclMinimum minimum point of cloud in PCL format 
         * @param pclMaximum maximum point of cloud in PCL format
         * @return centre point of cloud
         */
        Eigen::Vector4f centrePoint(Eigen::Vector4f pclMinimum, Eigen::Vector4f pclMaximum);

        /**
         * @brief returns centre point of cloud
         * @param pclCloud pointcloud for which the centre will be found
         * @return centre point of cloud
         */
        Eigen::Vector4f centrePoint(PointCloud::ConstPtr pclCloud);
        
        /**
         * @brief returns the overall dimensions of pointcloud
         * @param pclMinimum minimum point of cloud in PCL format 
         * @param pclMaximum maximum point of cloud in PCL format
         * @return size of cloud
         */
        Eigen::Vector4f boxSize(Eigen::Vector4f pclMinimum, Eigen::Vector4f pclMaximum);

        /**
         * @brief returns the overall dimensions of pointcloud 
         * @param pclCloud cloud for which overall size will be found
         * @return size of cloud
         */
        Eigen::Vector4f boxSize(PointCloud::ConstPtr pclCloud);
        
        /**
         * @brief I dont like this
         */
        void getBoundingBox(PointCloud::ConstPtr pclInputCloud, Eigen::Vector4f& pclMin, Eigen::Vector4f& pclMax);

        /**
         * @brief or this
         */
        void convertToPCLBox(Eigen::Vector4f size, Eigen::Vector4f centre, Eigen::Vector4f& pclMin, Eigen::Vector4f& pclMax);

        /**
         * @brief and this one but it might not be needed
         */
        void convertToROSBox(Eigen::Vector4f& pclMin, Eigen::Vector4f& pclMax, geometry_msgs::Pose* rosPose, geometry_msgs::Vector3* rosDimensions);

        /**
         * @brief Filter to remove points within set bounds
         * @param pclInputCloud cloud to be filtered
         * @param pclOutputCloud cloud for filtered points to be returned to
         * @param filterfield field (Axis) for the cloud to be filtered on
         * @param lowerLimit value for lower bounds of filter
         * @param upperLimit value for upper bounds of filter
         * @param negate negate action of filter (points outwith bounds will be kept) 
         */
        void passFilter(PointCloud::ConstPtr pclInputCloud, PointCloud::Ptr pclOutputCloud, std::string filterField, double lowerLimit, double upperLimit, bool negate);
       
        /**
         * @brief Segment cloud into small sections and removed lower points
         * @param pclInputCloud cloud to be filtered
         * @param pclOutputCloud cloud for filtered points to be returned to
         * @param xStepsize size of sections in x direction
         * @param yStepsize size of sections in y direction
         * @param delta increase to minimum point from which point are removed
         */
        void adaptiveGroundPlaneRemoval(PointCloud::ConstPtr pclInputCloud, PointCloud::Ptr pclOutputCloud, double xStepSize, double yStepSize, double delta);

        /**
         * @brief method for extraction of cone candidates
         * @param pclInputCloud cloud to be processed
         * @param clusters vector of cone candidates
         * @param maximumSeperation maximum distance points can be seperatec before disregarded from cluster
         * @param minimumClusterSize minimum number of points that can make up a cluster
         * @param maximumClusterSize maximum number of points that can make up a cluster
         */
        void eucCluster(PointCloud::ConstPtr pclInputCloud, std::vector<pcl::PointIndices>& clusters, double maximumSeperation, int minimumClusterSize, int maximumClusterSize);
        
        /**
         * @brief extract cone candidate points from cloud 
         * @param pclInputCloud cloud that points are extracted from (Should be as small as possible)
         * @param pclPointClusters vector of clusters that are to be extracted
         * @return vector of all restored cones
         */
        std::vector<PointCloud::Ptr> extractCandidates(PointCloud::ConstPtr pclInputCloud, std::vector<pcl::PointIndices>& pclPointClusters);
       
        /**
         * @brief restore cone candidates from unfiltered cloud
         * @param pclInputCloud cloud that cones will be restored from
         * @param coneCandidateClouds vector of all unrestored cones
         * @return vector of all restored cones
         */
        std::vector<PointCloud::Ptr> restoreCandidates(PointCloud::ConstPtr pclInputCloud, std::vector<PointCloud::Ptr>& coneCandidateClouds);
        
        /**
         * @brief Check cone is valid based on shape properties
         * @param pclInputCloud cone cloud that is to be checked for validity
         * @param lidarVerticalResolution the vertical resolution of the lidar (In Radians)
         * @param lidarHorizontalResolution the horizontal resolution of the lidar (In Radians)
         * @returns probability of cloud being cloud between 0.0 and 1.0 (Well at least it should)
         */
        double coneValiditycheck(PointCloud::ConstPtr pclInputCloud, double  lidarVerticalResolution, double lidarHorizontalResolution);
        /**
         * @brief take an input cloud and find all the cones within it
         */
        void findCones();

    private:
        /**
         * @brief Waits for publiser to be ready to publish messages
         */
        void waitForPublishers();

        /**
        * @brief Waits for subscribers to start receiving messages 
        */
        void waitForSubscribers();

        /**
         * @brief Callback for recieving lidar data
         * @param msg a lidar message
         */
        void lidarCallback(const sensor_msgs::PointCloud2ConstPtr& msg);


        //Some stuff
        //PCL Variables
        pcl::PassThrough<pcl::PointXYZ> passThroughFilter; //For FOV Trimming and Ground Plane Removal
        pcl::CropBox<Point> cropBoxFilter; //For Ground Plane Removal and Cone Restoration
        pcl::EuclideanClusterExtraction<Point> eucClustering; //For clustering

        //Function Variables
        double fovLowerLimit, fovUpperLimit; //FOV TRIMMING
        double gprXStep, gprYStep, gprDelta; //Ground Plane Removal
        double eucMaximumDistance; //Clustering
        int eucMinimumClusterSize, eucMaximumClusterSize;
        int boundingBoxSizeScaler, boundingBoxHeightScaler; 
        double minimumBoxSize; 
        double lidarVerticalRes, lidarHorizontalRes; //cone Validity Check
        int minimumCloudPercentage;
        std::string lidarTopic, coneTopic;
        int count_;

        ros::NodeHandle nh_;
        ros::Subscriber lidar_sub_;
        ros::Publisher cone_pub_;
        sensor_msgs::PointCloud2 global_cloud_;

        bool initialised_;
        bool lidar_subscriber_active_, cone_publisher_ready_;

};

#endif