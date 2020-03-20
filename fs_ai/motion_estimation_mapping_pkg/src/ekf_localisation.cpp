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
    private_nh_.param("ekf_gps_noise_x", sigmaZx, float(0.1));
    private_nh_.param("ekf_gps_noise_y", sigmaZy, float(0.1));
    private_nh_.param("ekf_imu_noise_yaw", sigmaZth, float(1.0 * M_PI / 180));

    //initialise control noise covariance matrix
    Q_ << powf(sigmaUx, 2), 0, 0, powf(sigmaUy, 2), 0, 0, powf(sigmaUth, 2);

    //initialise measurement noise covariance matrix
    R_ << powf(sigmaZx, 2), 0, 0, powf(sigmaZy, 2), 0, 0, powf(sigmaZth, 2);
}

/**
 * @brief
 * @param
 * @return
 */
void EKF::ekf_estimation_step(Eigen::Vector3f& xEst, Eigen::Matrix3f& pEst, const Eigen::Vector2f& u, float dt, float wb, const Eigen::Vector3f& z)
{
    //prediction step
    Eigen::Vector3f xPred;
    Eigen::Matrix3f pPred;
    Eigen::Matrix<float, 3, 2> jX;

    xPred << motion_model(xEst, u, dt, wb);
    jX << jacobian_position(xEst, u, dt, wb);
    pPred << jX * pEst * jX.transpose() + Q_;

    //correction (update) step
    Eigen::Matrix3f jZ;
    Eigen::Matrix3f innov_cov;
    Eigen::Matrix3f K;
    Eigen::Vector3f zPred;
    Eigen::Vector3f innov_seq;

    jZ << jacobian_measurement();
    innov_cov << jZ * pPred * jZ.transpose() + R_;
    K << pPred * jZ.transpose() * innov_cov.inverse();
    zPred << measurement_model(xPred, jZ);
    innov_seq << z - zPred;
    xEst = xPred + K * innov_seq;
    pEst = (Eigen::Matrix3f::Identity() - K * jZ) * pPred;
}

/**
 * @brief
 * @param
 * @return
 */
Eigen::Vector3f EKF::motion_model(const Eigen::Vector3f& xEst, const Eigen::Vector2f& u, float dt, float wb)
{
    //unpack the inputs
    float speed = u(0);
    float steering_angle = u(1);
    
    //calculate change from last estimate
    float delta_x = speed * cosf(xEst(2)) * dt;
    float delta_y = speed * sinf(xEst(2)) * dt;
    float delta_th = (speed * dt * tanf(steering_angle)) / wb;

    //calculate new estimates from motion model
    Eigen::Vector3f xPred;
    xPred << xEst(0) + delta_x, xEst(1) + delta_y, xEst(2) + delta_th;
    return xPred;
}

/**
 * @brief
 * @param
 * @return
 */
Eigen::Vector3f EKF::measurement_model(const Eigen::Vector3f& xPred, const Eigen::Matrix3f& jZ)
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
Eigen::Matrix<float, 3, 2> EKF::jacobian_position(const Eigen::Vector3f& x, const Eigen::Vector2f& u, float dt, float wb)
{
    float speed = u(0);
    float steering_angle = u(1);
    float current_yaw = x(2);

    Eigen::Matrix<float, 3, 2> jX;
    jX << dt * cosf(current_yaw), -speed * dt * sinf(current_yaw),
          dt * sinf(current_yaw), speed * dt * cosf(current_yaw),
          dt * tanf(steering_angle) / wb, speed * dt / (pow(cosf(steering_angle), 2) * wb);
    return jX;
}

/**
 * @brief Returns the jacobian matrix of the measurement model derived from GPS (x, y) and IMU (yaw) data input
 * @param
 * @return
 */
Eigen::Matrix3f EKF::jacobian_measurement()
{
    //no non-linearity since the values from GPS and IMU are provided by the sensors itself, not calculated from them
    Eigen::Matrix3f jZ;
    jZ << 1, 0, 0,
          0, 1, 0,
          0, 0, 1;
    return jZ;
}

