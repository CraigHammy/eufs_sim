#ifndef MCL_NODE_HEADER
#define MCL_NODE_HEADER

#include <ros/ros.h>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <vector>
#include "slam_utils.hpp"
#include <visualization_msgs/MarkerArray.h>
#include <nav_msgs/OccupancyGrid.h>

struct Observations
{
    std::vector<float> ranges;
    std::vector<float> angles;
};

struct MclParticle
{
    Eigen::Vector3f mu_;
    float weight_;
};

class MCL{
public:
    /**
     * @brief Constructor for Monte Carlo Localization class
     * @param nh A private ROS NodeHandle
     * @param map OccupancyGridMap message 
     */
    MCL(ros::NodeHandle* nh, const nav_msgs::OccupancyGrid::ConstPtr& map): private_nh_(nh), map_(*map) 
        {initialise();};

    /**
     * @brief Empty constructor for the MCL class
     */
    MCL() {};

     /**
     * @brief Monte Carlo Localisation weight calculation using real and predicted observation difference
     * @param xEst Eigen 3D vector describing the input from the EKF (prediction step)
     * @param scan Intensity array of the LaserScan message
     * @return The weight of the particle
     */
    float amcl_estimation_step(const Eigen::Vector3f& xEst, const std::vector<float>& scan);

private:
    //ROS private node
    ros::NodeHandle* private_nh_;

    //OccupancyGridMap
    nav_msgs::OccupancyGrid map_;

    //LaserScan parameters 
    float min_range_, max_range_;
    float min_angle_, max_angle_;
    float angle_increment_;

    //visualization variables
    visualization_msgs::MarkerArray filtered_scan_;
    visualization_msgs::MarkerArray predicted_scan_;
    ros::Publisher filtered_scan_pub_;
    ros::Publisher predicted_scan_pub_;

    /**
     * @brief Initialises parameters and variables needed for the Monte Carlo Localisation
     */
    void initialise();

    /**
     * @brief Filtering intensity array of LaserScan message to one point per cloud of laser detections
     * @param xEst Eigen 3D vector describing the input from the EKF (prediction step)
     * @param scan Intensity array of the LaserScan message
     * @return Filtered ranges and angles vector 
     */
    Observations filterScan(const Eigen::Vector3f xEst, const std::vector<float>& scan);

    /**
     * @brief Uses a transform listener to return the transformation matrix from the base to the velodyne frames
     * @return Eigen 3D transformation matrix 
     */
    Eigen::Matrix3f getBaseToVelodyneTF();

    /**
     * @brief Based on landmark map and current robot position, predict positions of cones 
     * @param xEst Eigen 3D vector describing the input from the EKF (prediction step)
     * @param num_intensities The size of the scans intensity vector
     * @return Predicted ranges and angles vector 
     */
    Observations predictObservation(const Eigen::Vector3f& xEst, int num_intensities);

    /**
     * @brief Subsamples the ranges array of the laser scan message 
     * @param scan Ranges array of the LaserScan message
     * @param step Values to skip in the scan array for every value added
     * @return Subsampled array 
     */
    std::vector<float> subsampleRanges(const std::vector<float>& scan, float step);
};

#endif