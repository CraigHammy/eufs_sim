#ifndef MCL_NODE_HEADER
#define MCL_NODE_HEADER

#include <ros/ros.h>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <vector>
#include "slam_utils.hpp"
#include <nav_msgs/OccupancyGrid.h>

class MCL{
public:
    /**
     * @brief Constructor for Monte Carlo Localization algorithm
     * @param nh A private ROS NodeHandle
     * @param map OccupancyGridMap message 
     */
    MCL(ros::NodeHandle* nh, const nav_msgs::OccupancyGrid::ConstPtr& map): private_nh_(nh), map_(*map) 
        {initialise();};

     /**
     * @brief Monte Carlo Localisation step: prediction and measurement update (correction)
     * @param xEst Eigen 3D vector describing the input from the EKF (prediction step)
     * @param scan Intensity array of the LaserScan message
     * @return Eigen 3D vector describing the pose of the robot after MCL prediction and measurement steps 
     */
    Eigen::Vector3f amcl_estimation_step(Eigen::Vector3f& xEst, const std::vector<float>& scan);

private:
    //ROS private node
    ros::NodeHandle* private_nh_;

    //OccupancyGridMap
    nav_msgs::OccupancyGrid map_;

    //LaserScan parameters 
    float min_range_, max_range_;
    float min_angle_, max_angle_;
    float angle_increment_;

    /**
     * @brief Initialises parameters and variables needed for the Monte Carlo Localisation
     */
    void initialise();

    /**
     * @brief Filtering intensity array of LaserScan message to one point per cloud of laser detections
     * @param xEst Eigen 3D vector describing the input from the EKF (prediction step)
     * @param scan Intensity array of the LaserScan message
     * @return Vector of Eigen 2D vectors each representing one point per cloud of close intensity values
     */
    std::vector<Eigen::Vector2f> MCL::filterScan(const Eigen::Vector3f xEst, const std::vector<float>& scan);

     /**
     * @brief Resamples the particles based on the updated weights from the correction step
     */
    void resampling();

    /**
     * @brief
     * @param
     * @result
     */
    void calculateWeight();

    /**
     * @brief Based on landmark map and current robot position, predict positions of cones 
     * @param xEst Eigen 3D vector describing the input from the EKF (prediction step)
     * @param num_intensities The size of the scans intensity vector
     * @return Predicted vector of Eigen 2D vector values 
     */
    std::vector<Eigen::Vector2f> MCL::predictObservation(const Eigen::Vector3f& xEst, int num_intensities);

    /**
     * @brief 
     * @param 
     * @result
     */
    void measurementUpdate();
};

#endif