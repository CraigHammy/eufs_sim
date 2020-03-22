#ifndef EKF_NODE_HEADER
#define EKF_NODE_HEADER

#include <ros/ros.h>
#include <Eigen/Core>
#include <Eigen/Dense>

/**
 * @brief Struct storing the motion model prediction and the measurement model correction robot states
 */
struct Estimates
{
    Eigen::Vector3f xPred;
    Eigen::Vector3f xEst;
};

// EXTENDED KALMAN FILTER CLASS
class EKF
{
public:
    /**
     * @brief Constructor for the Extende Kalman Filter class
     */
    EKF(ros::NodeHandle* private_nh): private_nh_(private_nh) {initialise();};

    /**
     * @brief Extended Kalman Filter step: prediction and measurement update (correction)
     * @param u Eigen 2D vector describing the robot control input: linear velocity and steering angle
     * @param dt Change in time (seconds) from previous step
     * @param wb Wheel base of the car-like robot (difference between centres of front and rear wheels)
     * @param z Eigen 3D vector describing the measurement: x and y GPS positions and IMU euler yaw orientation
     * @return Eigen 3D vectors describing the x, y and yaw pose values of the robot after the motion model and after EKF sensor fusion 
     */
    Estimates ekf_estimation_step(const Eigen::Vector2f& u, float dt, float wb, const  Eigen::Vector3f& z);

private:
    /**
     * @brief Initialises parameters and variables needed for the Extended Kalman Filter
     */
    void initialise();

    /**
     * @brief Using previous position and current robot control inputs, calculates the current predicted new position 
     * @param xEst Eigen 5D vector describing the previous state of the robot [x y yaw linear_vel steering_angle]'
     * @param u Eigen 2D vector describing the robot control input: linear velocity and steering angle
     * @param dt Change in time (seconds) from previous step
     * @param wb Wheel base of the car-like robot (difference between centres of front and rear wheels)
     * @return Eigen 5D vector describing the new predicted state of the robot [x y yaw linear_vel steering_angle]'
     */
    Eigen::Matrix<float, 5, 1> motion_model(const  Eigen::Matrix<float, 5, 1>& xEst, const Eigen::Vector2f& u, float dt, float wb);

    /**
     * @brief Maps the state vector from the prediction step to an observation
     * @param xPred Eigen 5D vector describing the predicted state of the robot
     * @param jZ Eigen 3x5 matrix describing Jacobian measurement matrix 
     * @return Eigen 3D vector representing the predicted state mapped as an observation 
     */
    Eigen::Vector3f measurement_model(const  Eigen::Matrix<float, 5, 1>& xPred, const  Eigen::Matrix<float, 3, 5>& jZ);

    /**
     * @brief Returns the Jacobian matrix of the motion model derived from the differential drive equations for a car
     * @param Eigen 5D vector representing the robot state vector
     * @param u Eigen 2D vector describing the robot control input: linear velocity and steering angle
     * @param dt Change in time (seconds) from previous step
     * @param wb Wheel base of the car-like robot (difference between centres of front and rear wheels)
     * @return Eigen 5x5 matrix representing the Jacobian control matrix 
     */
    Eigen::Matrix<float, 5, 5> jacobian_position(const  Eigen::Matrix<float, 5, 1>& x, const Eigen::Vector2f& u, float dt, float wb);
    
    /**
     * @brief Returns the jacobian matrix of the measurement model derived from GPS (x, y) and IMU (yaw) data input
     * @return Eigen 5x5 matrix representing the Jacobian measurement matrix 
     */
     Eigen::Matrix<float, 3, 5> jacobian_measurement();

    //private NodeHandle
    ros::NodeHandle* private_nh_;

    //parameters to generate the control and measurement noise matrices 
    float sigmaUx, sigmaUy, sigmaUth;
    float sigmaUv, sigmaUst;
    float sigmaZx, sigmaZy, sigmaZth;

    //mean and covariance of the Extended Kalman Filter
     Eigen::Matrix<float, 5, 1> mu_;
     Eigen::Matrix<float, 5, 5> sigma_;

    //motion model only estimate
    Eigen::Matrix<float, 5, 1> predEst_;

    //process and measurement noise matrices
     Eigen::Matrix<float, 5, 5> Q_;
     Eigen::Matrix3f R_;
};

#endif