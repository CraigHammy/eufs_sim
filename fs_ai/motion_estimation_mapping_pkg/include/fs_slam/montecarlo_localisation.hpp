#ifndef MCL_NODE_HEADER
#define MCL_NODE_HEADER

#include <ros/ros.h>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <vector>

class MCL{
public:
    /**
     * @brief Constructor for Monte Carlo Localization algorithm
     * @param nh A private ROS NodeHandle
    MCL(ros::NodeHandle* nh): private_nh_(nh) {};

    Eigen::Vector3f amcl_estimation_step(Eigen::Vector3f& xEst, const std::vector<float>& scan);

private:
    ros::NodeHandle* private_nh_;

    void filterScan(const std::vector<float>& scan);

     /**
     * @brief Resamples the particles based on the updated weights from the correction step
     */
    void resampling();

     /**
     * @brief Calculates the squared norm of the difference between the observations based on the map and on sensor data
     * @return Squared norm of difference between observations based on map and on sensor data  
     */
    float squaredNormDifference();

    void calculateWeight();
    void predictObservation();
    void measurementUpdate();
};

#endif