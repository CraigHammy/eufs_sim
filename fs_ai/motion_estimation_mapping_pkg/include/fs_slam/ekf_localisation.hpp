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
    Eigen::Vector3f ekf_estimation_step(const Eigen::Vector2f& u, float dt, float wb, const  Eigen::Vector3f& z);

    /**
     * @brief
     * @param
     * @return
     */
    Eigen::Matrix<float, 5, 5> jacobian_position(const  Eigen::Matrix<float, 5, 1>& x, const Eigen::Vector2f& u, float dt, float wb);

    /**
     * @brief
     * @param
     * @return
     */
     Eigen::Matrix<float, 3, 5> jacobian_measurement();

    /**
     * @brief
     * @param
     * @return
     */
     Eigen::Matrix<float, 5, 1> motion_model(const  Eigen::Matrix<float, 5, 1>& xEst, const Eigen::Vector2f& u, float dt, float wb);

    /**
     * @brief
     * @param
     * @return
     */
     Eigen::Vector3f measurement_model(const  Eigen::Matrix<float, 5, 1>& xPred, const  Eigen::Matrix<float, 3, 5>& jZ);


private:
    //private node handle
    ros::NodeHandle private_nh_;

    //parameters 
    float sigmaUx;
    float sigmaUy;
    float sigmaUth;
    float sigmaUv;
    float sigmaUst;
    float sigmaZx;
    float sigmaZy;
    float sigmaZth;

    //mean and covariance of the Extended Kalman Filter
     Eigen::Matrix<float, 5, 1> mu_;
     Eigen::Matrix<float, 5, 5> sigma_;

    //process and measurement noise 
     Eigen::Matrix<float, 5, 5> Q_;
     Eigen::Matrix3f R_;
};

#endif