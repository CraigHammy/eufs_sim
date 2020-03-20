#include <ros/ros.h>
#include "ekf_localisation.hpp"
#include <Eigen/Core>
#include <Eigen/Dense>

/**
 * @brief
 * @param
 * @return
 */
void EKF::initialise()
{
    //initialise parameters from server 
    private_nh_.param("ekf_control_noise_x", sigmaUx, float(0.1));
    private_nh_.param("ekf_control_noise_y", sigmaUy, float(0.1));
    private_nh_.param("ekf_control_noise_yaw", sigmaUth, float(1.0 * M_PI / 180));
    private_nh_.param("ekf_control_noise_vel", sigmaUv, float(1.0));
    private_nh_.param("ekf_control_noise_steer", sigmaUst, float(1.0 * M_PI / 180));
    private_nh_.param("ekf_gps_noise_x", sigmaZx, float(0.1));
    private_nh_.param("ekf_gps_noise_y", sigmaZy, float(0.1));
    private_nh_.param("ekf_imu_noise_yaw", sigmaZth, float(1.0 * M_PI / 180));

    //initialise control noise covariance matrix
    Q_ << powf(sigmaUx, 2), 0, 0, 0, 0, 
        0, powf(sigmaUy, 2), 0, 0, 0, 
        0, 0, powf(sigmaUth, 2), 0, 0,
        0, 0, 0, powf(sigmaUv, 2), 0,
        0, 0, 0, 0, powf(sigmaUst, 2);

    //initialise measurement noise covariance matrix
    R_ << powf(sigmaZx, 2), 0, 0, 0, powf(sigmaZy, 2), 0, 0, 0, powf(sigmaZth, 2);

    mu_ << Eigen::Matrix<float, 5, 1>::Zero();
    sigma_ << Eigen::Matrix<float, 5, 5>::Identity();
}

/**
 * @brief
 * @param
 * @return
 */
Eigen::Vector3f EKF::ekf_estimation_step(const Eigen::Vector2f& u, float dt, float wb, const  Eigen::Vector3f& z)
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
    sigma_ = (Eigen::Matrix<float, 5, 5>::Identity() - K * jZ) * pPred;

    Eigen::Vector3f position;
    position << mu_(0), mu_(1), mu_(2);
    return position;
}

/**
 * @brief
 * @param
 * @return
 */
 Eigen::Matrix<float, 5, 1> EKF::motion_model(const  Eigen::Matrix<float, 5, 1>& xEst, const Eigen::Vector2f& u, float dt, float wb)
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
 * @brief
 * @param
 * @return
 */
 Eigen::Vector3f EKF::measurement_model(const Eigen::Matrix<float, 5, 1>& xPred, const Eigen::Matrix<float, 3, 5>& jZ)
{
    Eigen::Vector3f z;
    z << jZ * xPred;
    return z;
}

/**
 * @brief Returns the Jacobian matrix of the motion model derived from the differential drive equations for a car
 * @param
 * @return
 */
Eigen::Matrix<float, 5, 5> EKF::jacobian_position(const Eigen::Matrix<float, 5, 1>& x, const Eigen::Vector2f& u, float dt, float wb)
{
    float speed = u(0);
    float steering_angle = u(1);
    float current_yaw = x(2);

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
 * @param
 * @return
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

