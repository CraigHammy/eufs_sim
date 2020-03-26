#ifndef FASTSLAM_PARTICLE_HEADER
#define FASTSLAM_PARTICLE_HEADER

#include "slam_utils.hpp"
#include <Eigen/Dense>
#include <perception_pkg/Cone.h>
#include <vector>
#include <sensor_msgs/PointCloud.h>
#include <Eigen/Core>
#include "boost/random.hpp"
#include "ekf_localisation.hpp"
#include "montecarlo_localisation.hpp"

class Particle
{
public:
    Particle(int num_landmarks, EKF* ekf, MCL* mcl): num_lms(num_landmarks), mu_(Eigen::Vector3f::Zero()), sigma_(Eigen::Matrix3f::Identity()), ekf_(*ekf), mcl_(*mcl) {};
    Particle(EKF* ekf): num_lms(INT_MAX), mu_(Eigen::Vector3f::Zero()), mu_pred_(Eigen::Vector3f::Zero()), sigma_(Eigen::Matrix3f::Identity()), ekf_(*ekf) {};
    Particle(const Eigen::Vector3f mu, const Eigen::Vector3f mu_pred, Eigen::Matrix3f sigma, EKF* ekf, MCL *mcl): num_lms(INT_MAX), mu_(mu), mu_pred_(mu_pred), sigma_(sigma), ekf_(*ekf), mcl_(*mcl) {};

    /**
     * @brief State estimation prediction based on odometric and robot control state data
     * @param current_pose An Eigen 3D vector, describing the current pose estimate  
     * @param speed The current linear speed of the robot
     * @param steer The current steering angle of the robot
     * @param wb The wheelbase of the differential-drive robot, the distance between the centres of the front and rear wheels
     * @param delta Time step from last known pose 
     * @param Gx Eigen 3x3 Jacobian with respect to location 
     * @param Gv Eigen 3x2 Jacobian matrix with respect to control
     * @param Q Eigen 2x2 covariance matrix of control noise
     */
    void motionUpdate(const Eigen::Vector3f& current_pose, float speed, float steer, float wb, float delta, Eigen::Vector3f z, const Eigen::Matrix3f& Gx, const Eigen::Matrix<float, 3, 2>& Gu, const Eigen::Matrix2f& Q);

    /**
     * @brief Determines whether the observation is an unknown or known feature, and performs data association on it if is known
     * @param z A Cone message 
     * @param current_pose A RobotPose object
     * @param R Eigen 2x2 covariance matrix of measurement noise
     * @param slam_phase The phase (mapping or localization) the car is currently executing
     */
    void measurementUpdate(const perception_pkg::Cone& z, const Eigen::Matrix2f& R, SLAM_PHASE slam_phase);
        
    /**
     * @brief Generates a proposal distribution using the observations of the known features and samples a pose from it
     * @param known_features Vector of DataAssociation values corresponding to known features from newly observed cones 
     * @param R Eigen 2x2 covariance matrix of measurement noise
     */
    void proposalSampling(const std::vector<DataAssociation> known_features, const Eigen::Matrix2f& R);
    
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
     * @param ob Eigen 2D vector describing the euclidean distance and yaw angle to the landmark
     * @param colour The colour of the cone landmark
     * @param R Eigen 2x2 covariance matrix of measurement noise
     */
    Eigen::Vector2f addNewLandmark(const Eigen::Vector2f& ob, const std::string& colour, const Eigen::Matrix2f& R);

    /**
     * @brief Converts a vector of Landmarks into Points, and add them to PointCloud
     * @param points Vector to place the landmark positions in 
     * @param cloud PointCloud used to visualize landmarks 
     */
    void convertToPoints(std::vector<geometry_msgs::Point>& points, sensor_msgs::PointCloud& cloud);

    //FastSLAM2.0 particle variables 
    float weight_;      
    Eigen::Vector3f mu_;
    Eigen::Matrix3f sigma_;
    std::vector<Landmark> landmarks_;
    std::vector<DataAssociation> known_features_, unknown_features_;

    Eigen::Vector3f mu_pred_;
    EKF ekf_;
    MCL mcl_;

private:
    int num_lms;

    /**
     * @brief Checks if two landmarks are the same based on their colour and euclidean distance between them
     * @param lm1 First Landmark object
     * @param lm2 Second Landmark object
     * @return True if any stored landmark has the same colour and distance less than 20cm from lm
     */
    bool sameLandmark(const Landmark& lm1, const Landmark& lm2);

    /**
     * @brief Generates data association value: the higher the value, the more probable that the observation corresponds to the landmark
     * @param lm_innov_cov Eigen 2x2 innovation covariance matrix of observed landmark
     * @param lm_innov_mean Eigen 2x1 innovation mean vector of observed landmark
     * @param Hv Eigen 2x3 Jacobian matrix with respect to the robot location
     * @param zt Eigen 2x1 measurement vector from robot pose to landmark
     * @param z A Cone message 
     * @param lm A Landmark object
     * @param R Eigen 2x2 covariance matrix of measurement noise
     * @return Data association value
     */
    float dataAssociations(const Eigen::Matrix2f& lm_innov_cov, const Eigen::Vector2f& lm_innov_mean, const Eigen::Matrix<float, 2, 3>& Hv, const Eigen::Vector2f& zt, const perception_pkg::Cone& z, const Landmark& lm, const Eigen::Matrix2f& R);

    /**
     * @brief One step towards the generation of the proposal distribution using only one of the known feature data associations 
     * @param lm_innov_cov Eigen 2x2 innovation covariance matrix of observed landmark
     * @param lm_innov_mean Eigen 2x1 innovation mean vector of observed landmark
     * @param Hv Eigen 2x3 Jacobian matrix with respect to the robot location
     */
    void proposalDistribution(const Eigen::Matrix2f& lm_innov_cov, const Eigen::Vector2f& lm_innov_mean, const Eigen::Matrix<float, 2, 3>& Hv);
};

#endif
