#include <ros/ros.h>
#include "ekf_localisation.hpp"
#include <Eigen/Core>
#include <Eigen/Dense>

/**
 * @brief Initialises parameters and variables needed for the Extended Kalman Filter
 */
void EKF::initialise()
{
    //initialise parameters from server 
    private_nh_.param("ekf_control_noise_x", sigmaUx);
    private_nh_.param("ekf_control_noise_y", sigmaUy);
    private_nh_.param("ekf_control_noise_yaw", sigmaUth);
    private_nh_.param("ekf_control_noise_vel", sigmaUv);
    private_nh_.param("ekf_control_noise_steer", sigmaUst);
    private_nh_.param("ekf_gps_noise_x", sigmaZx);
    private_nh_.param("ekf_gps_noise_y", sigmaZy);
    private_nh_.param("ekf_imu_noise_yaw", sigmaZth);

    //initialise control noise covariance matrix
    Q_ << powf(sigmaUx, 2), 0, 0, 0, 0, 
        0, powf(sigmaUy * M_PI / 180, 2), 0, 0, 0, 
        0, 0, powf(sigmaUth, 2), 0, 0,
        0, 0, 0, powf(sigmaUv, 2), 0,
        0, 0, 0, 0, powf(sigmaUst * M_PI / 180, 2);

    //initialise measurement noise covariance matrix
    R_ << powf(sigmaZx, 2), 0, 0, 0, powf(sigmaZy, 2), 0, 0, 0, powf(sigmaZth * M_PI / 180, 2);

    //initialise mean and covariance of the Extended Kalman Filter 
    mu_ << Eigen::Matrix<float, 5, 1>::Zero();
    sigma_ << Eigen::Matrix<float, 5, 5>::Identity();
}

/**
 * @brief Extended Kalman Filter step: prediction and measurement update (correction)
 * @param u Eigen 2D vector describing the robot control input: linear velocity and steering angle
 * @param dt Change in time (seconds) from previous step
 * @param wb Wheel base of the car-like robot (difference between centres of front and rear wheels)
 * @param z Eigen 3D vector describing the measurement: x and y GPS positions and IMU euler yaw orientation
 * @return Eigen 3D vectors describing the x, y and yaw pose values of the robot after the motion model and after EKF sensor fusion 
 */
Estimates EKF::ekf_estimation_step(const Eigen::Vector2f& u, float dt, float wb, const Eigen::Vector3f& z)
{
    //prediction step
    Eigen::Matrix<float, 5, 1> xPred;
    Eigen::Matrix<float, 5, 5> pPred;
    Eigen::Matrix<float, 5, 5> jX;

    xPred << motion_model(mu_, u, dt, wb);
    jX << jacobian_position(mu_, u, dt, wb);
    pPred << jX * sigma_ * jX.transpose() + Q_;

    //correction (update) step
    Eigen::Matrix<float, 3, 5> jZ;
    Eigen::Matrix<float, 3, 3> innov_cov;
    Eigen::Matrix<float, 5, 3> K;
    Eigen::Matrix<float, 3, 1> zPred;
    Eigen::Matrix<float, 3, 1> innov_seq;

    jZ << jacobian_measurement();
    innov_cov << jZ * pPred * jZ.transpose() + R_;
    K << pPred * jZ.transpose() * innov_cov.inverse();
    zPred << measurement_model(xPred, jZ);
    innov_seq << z - zPred;
    mu_ = xPred + K * innov_seq;
    std::cout << "updated mean\n" << xPred << std::endl;
    sigma_ = (Eigen::Matrix<float, 5, 5>::Identity() - K * jZ) * pPred;

    //create 3D vectors to pass prediction and correction state vectors to the FastSLAM2.0 algorithm 
    Estimates e;
    Eigen::Vector3f xEst, xPred3D;
    xEst << mu_(0), mu_(1), mu_(2);
    xPred3D << xPred(0), xPred(1), xPred(2);
    e.xPred = xPred3D;
    e.xEst = xEst;
    return e;
}

/**
 * @brief Using previous position and current robot control inputs, calculates the current predicted new position 
 * @param xEst Eigen 5D vector describing the previous state of the robot [x y yaw linear_vel steering_angle]'
 * @param u Eigen 2D vector describing the robot control input: linear velocity and steering angle
 * @param dt Change in time (seconds) from previous step
 * @param wb Wheel base of the car-like robot (difference between centres of front and rear wheels)
 * @return Eigen 5D vector describing the new predicted state of the robot [x y yaw linear_vel steering_angle]'
 */
Eigen::Matrix<float, 5, 1> EKF::motion_model(const Eigen::Matrix<float, 5, 1>& xEst, const Eigen::Vector2f& u, float dt, float wb)
{
    //unpack the inputs
    float speed = u(0);
    float steering_angle = u(1);
    
    //calculate change from last estimate
    float delta_x = speed * cosf(xEst(2)) * dt;
    float delta_y = speed * sinf(xEst(2)) * dt;
    float delta_th = (speed * dt * tanf(steering_angle)) / wb;

    //calculate new estimates from motion model
    Eigen::Matrix<float, 5, 1> xPred;
    xPred << xEst(0) + delta_x, xEst(1) + delta_y, xEst(2) + delta_th, speed, steering_angle;
    return xPred;
}

/**
 * @brief Maps the state vector from the prediction step to an observation
 * @param xPred Eigen 5D vector describing the predicted state of the robot
 * @param jZ Eigen 3x5 matrix describing Jacobian measurement matrix 
 * @return Eigen 3D vector representing the predicted state mapped as an observation 
 */
Eigen::Vector3f EKF::measurement_model(const Eigen::Matrix<float, 5, 1>& xPred, const Eigen::Matrix<float, 3, 5>& jZ)
{
    Eigen::Vector3f z;
    z << jZ * xPred;
    return z;
}

/**
 * @brief Returns the Jacobian matrix of the motion model derived from the differential drive equations for a car
 * @param Eigen 5D vector representing the robot state vector
 * @param u Eigen 2D vector describing the robot control input: linear velocity and steering angle
 * @param dt Change in time (seconds) from previous step
 * @param wb Wheel base of the car-like robot (difference between centres of front and rear wheels)
 * @return Eigen 5x5 matrix representing the Jacobian control matrix 
 */
Eigen::Matrix<float, 5, 5> EKF::jacobian_position(const Eigen::Matrix<float, 5, 1>& x, const Eigen::Vector2f& u, float dt, float wb)
{
    //unpack inputs 
    float speed = u(0);
    float steering_angle = u(1);
    float current_yaw = x(2);

    //create 5x5 Jacobian matrix from state vector [x y yaw linear_vel steering_angle]'
    Eigen::Matrix<float, 5, 5> jX;
    jX << 1, 0, 0,  dt * cosf(current_yaw), -speed * dt * sinf(current_yaw),
          0, 1, 0, dt * sinf(current_yaw), speed * dt * cosf(current_yaw),
          0, 0, 1, dt * tanf(steering_angle) / wb, speed * dt / (pow(cosf(steering_angle), 2) * wb),
          0, 0, 0, 1, 0,
          0, 0, 0, 0, 1;
    return jX;
}

/**
 * @brief Returns the jacobian matrix of the measurement model derived from GPS (x, y) and IMU (yaw) data input
 * @return Eigen 5x5 matrix representing the Jacobian measurement matrix 
 */
Eigen::Matrix<float, 3, 5> EKF::jacobian_measurement()
{
    //no non-linearity since the values from GPS and IMU are provided by the sensors itself, not calculated from them
    Eigen::Matrix<float, 3, 5> jZ;
    jZ << 1, 0, 0, 0, 0,
          0, 1, 0, 0, 0,
          0, 0, 1, 0, 0;
    return jZ;
}

