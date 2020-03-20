#ifndef EKF_NODE_HEADER
#define EKF_NODE_HEADER

#include <ros/ros.h>
#include <Eigen/Core>
#include <Eigen/Dense>

class EKF
{
public:
    EKF(ros::NodeHandle* private_nh): private_nh_(*private_nh) {};

    /**
     * @brief
     * @param
     * @return
     */
    void initialise();

    /**
     * @brief
     * @param
     * @return
     */
    void ekf_estimation_step(Eigen::Vector3f& xEst, Eigen::Matrix3f& pEst, const Eigen::Vector2f& u, float dt, float wb, const Eigen::Vector3f& z);

    /**
     * @brief
     * @param
     * @return
     */
    Eigen::Matrix<float, 3, 2> jacobian_position(const Eigen::Vector3f& x, const Eigen::Vector2f& u, float dt, float wb);

    /**
     * @brief
     * @param
     * @return
     */
    Eigen::Matrix3f jacobian_measurement();

    /**
     * @brief
     * @param
     * @return
     */
    Eigen::Vector3f motion_model(const Eigen::Vector3f& xEst, const Eigen::Vector2f& u, float dt, float wb);

    /**
     * @brief
     * @param
     * @return
     */
    Eigen::Vector3f measurement_model(const Eigen::Vector3f& xPred, const Eigen::Matrix3f& jZ);


private:
    //private node handle
    ros::NodeHandle private_nh_;

    //parameters 
    float sigmaUx;
    float sigmaUy;
    float sigmaUth;
    float sigmaZx;
    float sigmaZy;
    float sigmaZth;

    //mean and covariance of the Extended Kalman Filter
    Eigen::Vector3f mu_;
    Eigen::Matrix3f sigma_;

    //process and measurement noise 
    Eigen::Matrix3f Q_;
    Eigen::Matrix3f R_;
};

#endif