#ifndef FASTSLAM_PARTICLE_HEADER
#define FASTSLAM_PARTICLE_HEADER

#include "slam_utils.hpp"
#include <Eigen/Dense>
#include <perception_pkg/Cone.h>
#include <vector>
#include <Eigen/Core>
#include "boost/random.hpp"

class Particle
{
public:
    Particle(int num_landmarks): num_lms(num_landmarks) {};
    Particle(): num_lms(20), mu_(Eigen::Vector3f::Zero()), sigma_(Eigen::Matrix3f::Identity()) {};

    /**
     * @brief State estimation prediction based on odometric and robot control state data
     * @param current_pose An Eigen vector, describing the current pose estimate  
     * @param speed The current linear speed of the robot
     * @param steer The current steering angle of the robot
     * @param wb The wheelbase of the differential-drive robot, the distance between the centres of the front and rear wheels
     * @param delta Time step from last known pose 
     * @param Gx Eigen 3x3 Jacobian with respect to location 
     * @param Gv Eigen 3x2 Jacobian matrix with respect to control
     * @param Q Eigen 2x2 covariance matrix of control noise
     */
    void motionUpdate(const Eigen::Vector3f& current_pose, float speed, float steer, float wb, float delta, const Eigen::Matrix3f& Gx, const Eigen::Matrix<float, 3, 2>& Gu, const Eigen::Matrix2f& Q);

    /**
     * @brief Perform correction step on a particle
     * @param observations List of Cone messages 
     * @param current_pose An Eigen vector 
     * @param slam_phase The phase (mapping or localization) the car is currently executing 
     */
    void measurementUpdate(const perception_pkg::Cone& z, Eigen::Vector3f& current_pose, const Eigen::Matrix2f& R, const Eigen::Matrix2f& Q, SLAM_PHASE slam_phase);
        
    /**
     * @brief Generates a proposal distribution using an observation and samples a pose from it
     * @param lm_innov_cov Eigen 2x2 innovation covariance matrix of observed landmark
     * @param lm_innov_mean Eigen 2x1 innovation mean vector of observed landmark
     * @param Hv Eigen 2x2 Jaocbian matrix with respect to robot location
     * @param zt Eigen 2x1 measurement vector from robot pose to landmark 
     * @return Data association value
     */
    void proposalSampling(const std::vector<DataAssociation> known_features, const Eigen::Matrix2f& R);

    /**
     * @brief
     * @param
     * @return
     */
    void proposalDistribution(const Eigen::Matrix2f& lm_innov_cov, const Eigen::Vector2f& lm_innov_mean, const Eigen::Matrix<float, 2, 3>& Hv);
    
    /**
     * @brief Calculates weight of particle given a landmark and an observation
     * @param innov_covariance Covariance innovation from landmark EKF calculation
     * @return innov_mean Mean innovation from landmark EKF calculation
     * @return The calculated weight 
     */
    float calculateWeight(const Eigen::Vector2f& innov_mean, const Eigen::Matrix2f& lm_sigma, const Eigen::Matrix2f& Hl, const Eigen::Matrix<float, 2, 3>& Hv, const Eigen::Matrix2f& R);
    
     /**
     * @brief Updates the state and covariance of the landmark 
     * @param lm A Landmark object
     * @param H Jacobian matrix of landmark observation
     * @param innov_cov Covariance innovation from landmark EKF calculation
     */
    void updateLandmarkWithCholesky(Landmark& lm, const Eigen::Matrix2f& H, const Eigen::Matrix2f& innov_cov, const Eigen::Vector2f& innov_mean);

    /**
     * @brief Adds new landmark to the list of landmarks associated to the particle
     * @param ob A 2D vector describing the euclidean distance and yaw angle to the landmark 
     * @param colour The colour of the cone landmark 
     * @param R Eigen 2x2 covariance matrix of measurement noise
     */
    Eigen::Vector2f addNewLandmark(const Eigen::Vector2f& ob, const std::string& colour, const Eigen::Matrix2f& R);

    //these should be private-> try working with friend classes and method (protected section??)
    float weight_;      
    Eigen::Vector3f mu_;
    Eigen::Matrix3f sigma_;
    //std::vector<Landmark> landmarks_(int(num_lms));
    std::vector<Landmark> landmarks_;
    std::vector<DataAssociation> known_features_, unknown_features_;

private:
    int num_lms;

    /**
     * @brief Updates the state and covariance of the landmark 
     * @param lm A Landmark object
     * @param H Jacobian matrix of landmark observation
     * @param innov_cov Covariance innovation from landmark EKF calculation
     */
    void updateLandmark(Landmark& lm, const Eigen::Matrix2f& H, const Eigen::Matrix2f& innov_cov, const Eigen::Vector2f& innov_mean);

    /**
     * @brief Checks if current landmark is the same as a previously observed landmark
     * @param lm A Landmark object
     * @param landmarks_to_check List of Landmark objects to check against 
     * @return True if any stored landmark has the same colour and distance less than 20cm from lm
     */
    bool sameLandmark(const Landmark& lm, const std::vector<Landmark>& landmarks_to_check);

    /**
     * 
     */
    float dataAssociations(const Eigen::Matrix2f& lm_innov_cov, const Eigen::Vector2f& lm_innov_mean, const Eigen::Matrix<float, 2, 3>& Hv, const Eigen::Vector2f& zt, const perception_pkg::Cone& ob, const Landmark& lm, const Eigen::Matrix2f& R);

};

#endif
