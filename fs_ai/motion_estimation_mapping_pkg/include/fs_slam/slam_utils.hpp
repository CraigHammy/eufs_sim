#ifndef UTILITIES_HEADER
#define UTILITIES_HEADER

#include <Eigen/Dense>
#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <Eigen/Core>

enum SLAM_PHASE {
    MAP_BUILDING,
    LOCALIZATION
};

struct RobotPose
{
    //odom variables
    Eigen::Vector3f odom_pose_;
    Eigen::Vector3f odom_twist_;
};

struct RobotCommand
{
    //linear velocity and steering angle
    float speed;
    float steering_angle;
};

struct Landmark
{
    Eigen::Vector2f mu_;
    Eigen::Matrix2f sigma_;
    std::string colour_;
};


struct Innovation
{
    Eigen::Vector2f predicted_observation;
    Eigen::Matrix<float, 2, 3> vehicle_jacobian;
    Eigen::Matrix2f feature_jacobian;
    Eigen::Matrix2f innovation_covariance;
};

struct DataAssociation
{
    Eigen::Vector2f measurement;
    int landmark_id;
    std::string colour;
};

static const double NEW_LANDMARK_THRESH = 0.0000001;

/**
 * @brief Computes landmark EKF predicted observation, Jacobians wrt to landmark and robot locations and landmark innovation covariance
 * @param mean Eigen 3D vector describing the Mean of the landmark EKF 
 * @param sigma Eigen 3x3 matrix describing the covariance of the landmark EKF 
 * @param landmark A Landmark object
 * @param R Eigen 2x2 covariance matrix of measurement noise
 * @return Innovation variable which includes landmark EKF predicted observation, Jacobians wrt to landmark and robot locations and landmark innovation covariance
 */
Innovation computeInnovations(const Eigen::Vector3f& mean, const Eigen::Matrix3f& sigma, const Landmark& landmark, const Eigen::Matrix2f& R);

/**
 * @brief Wraps the angle between -180 and +180 in radians
 * @param angle Angle value as float
 * @return Wrapped angle in radians 
 */
float angleWrap(float angle);

/**
 * @brief Generates Eigen array of cumulative sums 
 * @param weights Eigen array of floats
 * @return Resulting Eigen array of cumulative weights 
 */
Eigen::ArrayXf cumulativeSum(Eigen::ArrayXf weights);

/**
 * @brief Retrieve measurement containing distance and angle to landmark
 * @param sensor_reading A Point message describing a sensor reading from the Velodyne
 * @return An Eigen 2d vector describing the distance and angle to the landmark
 */
Eigen::Vector2f getMeasurement(const geometry_msgs::Point::ConstPtr& observation);

#endif
