#include "particle.hpp"
#include "slam_utils.hpp"
#include <Eigen/Dense>
#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <std_msgs/Header.h>
#include <eufs_msgs/WheelSpeedsStamped.h>
#include <Eigen/Cholesky>
#include <Eigen/Core>
#include <cmath>
#include <vector>
#include <algorithm>
#include <string>
#include <iostream>
#include <stdint.h>
#include <ctime>
#include <fstream>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <sensor_msgs/ChannelFloat32.h>

std::time_t now = std::time(0);
boost::random::mt19937 rng(static_cast<uint32_t>(now));

/**
 * @brief State estimation prediction based on odometric and robot input control data
 * @param current_pose An Eigen 3D vector, describing the current pose estimate
 * @param speed The current linear speed of the robot
 * @param steer The current steering angle of the robot
 * @param wb The wheelbase of the differential-drive robot, the distance between the centres of the front and rear wheels
 * @param delta Time step from last known pose
 * @param Gx Eigen 3x3 Jacobian with respect to location
 * @param Gv Eigen 3x2 Jacobian matrix with respect to control
 * @param Q Eigen 2x2 covariance matrix of control noise
 */
void Particle::motionUpdate(const Eigen::Vector3f& current_pose, float speed, float steer, float wb, float delta, Eigen::Vector3f z, const Eigen::Matrix3f& Gx, const Eigen::Matrix<float, 3, 2>& Gu, const Eigen::Matrix2f& Q)
{
    //calculate the difference from the last known pose
    //float delta_x = speed * cosf(current_pose(2)) * delta;
    //float delta_y = speed * sinf(current_pose(2)) * delta;
    //float delta_th = (speed * delta * tanf(steer)) / wb;

    //predict the particle mean
    //mu_(0) += delta_x;
    //mu_(1) += delta_y;
    //mu_(2) += delta_th;

    //predict the particle covariance
    //sigma_ = Gx * sigma_ * Gx.transpose() + Gu * Q * Gu.transpose();
}


/**
 * @brief Determines whether the observation is an unknown or known feature, and performs data association on it if is known
 * @param z A Cone message 
 * @param current_pose A RobotPose object
 * @param R Eigen 2x2 covariance matrix of measurement noise
 * @param slam_phase The phase (mapping or localization) the car is currently executing
 */
void Particle::measurementUpdate(const perception_pkg::Cone& z, const Eigen::Matrix2f& R, SLAM_PHASE slam_phase)
{

    boost::shared_ptr<geometry_msgs::Point> z_ptr(new geometry_msgs::Point(z.location));
    Eigen::Vector2f measurement(getMeasurement(z_ptr));

    //assume the observations received are all unique and not repeated
    if(landmarks_.empty())
    {
        //add the first landmark to the particle
        addNewLandmark(measurement, z.colour, R);
        return;
    }

    //data association variables
    Eigen::Matrix<float, 2, 3> best_Hv = Eigen::Matrix<float, 2, 3>::Zero();
    Eigen::Matrix2f best_Hl = Eigen::Matrix2f::Zero();
    Eigen::Matrix2f best_innov_cov = Eigen::Matrix2f::Zero();
    Eigen::Vector2f best_innov_mean = Eigen::Vector2f::Zero();
    float best_p = 0;
    int best_arg = 0;

    //iterating over all landmarks
    int i = 0;
    std::vector<Landmark>::const_iterator lm;
    for(lm = landmarks_.begin(); lm != landmarks_.end(); ++lm, ++i)
    {
        Innovation innovation(computeInnovations(mu_, sigma_, *lm, R));
        Eigen::Matrix<float, 2, 3> Hv(innovation.vehicle_jacobian);
        Eigen::Matrix2f Hl(innovation.feature_jacobian);
        Eigen::Vector2f zt(innovation.predicted_observation);
        Eigen::Matrix2f lm_innov_cov(innovation.innovation_covariance);

        Eigen::Vector2f lm_innov_mean(measurement - zt);
        lm_innov_mean(1) = angleWrap(lm_innov_mean(1));

        //perform data association to distinguish new features, and assigning correct observations to known features
        float p = dataAssociations(lm_innov_cov, lm_innov_mean, Hv, zt, z, *lm, R);

        if ((i == 0) || (p > best_p))
        {
            best_p = p;
            best_arg = i;
            best_Hv = Hv;
            best_innov_cov = lm_innov_cov;
            best_innov_mean = lm_innov_mean;
        }
    }

    Eigen::Vector2f map_feature(z.location.x + mu_(0), z.location.y + mu_(1));
    if ((best_p < NEW_LANDMARK_THRESH) && ((map_feature - landmarks_.at(best_arg).mu_).norm() > 1.25))
    {
        DataAssociation d;
        d.measurement = measurement;
        d.landmark_id = best_arg;
        unknown_features_.push_back(d);
    }
    else
    {
        DataAssociation d;
        d.measurement = measurement;
        d.landmark_id = best_arg;
        known_features_.push_back(d);
    }

    if (slam_phase == MAP_BUILDING) {
        ;//do stuff
    }
    else if (slam_phase == LOCALIZATION) {
        ;//do stuff
    }
}

/**
 * @brief Adds new landmark to the list of landmarks associated to the particle
 * @param ob Eigen 2D vector describing the euclidean distance and yaw angle to the landmark
 * @param colour The colour of the cone landmark
 * @param R Eigen 2x2 covariance matrix of measurement noise
 */
Eigen::Vector2f Particle::addNewLandmark(const Eigen::Vector2f& ob, const std::string& colour, const Eigen::Matrix2f& R)
{
    Landmark landmark;
    //sine and cosine of yaw angle
    float s = sinf(angleWrap(mu_(2) + ob(1)));
    float c = cosf(angleWrap(mu_(2) + ob(1)));

    //x and y values of landmark respect to robot pose (maybe do it in map frame)
    float lm_x = mu_(0) + ob(0)*c;
    float lm_y = mu_(1) + ob(0)*s;

    //landmark state
    landmark.mu_ << lm_x, lm_y;

    //landmark covariance
    Eigen::Matrix2f G;
    G << c, -ob(0) * s, s, ob(0) * c;
    landmark.sigma_  << G * R * G.transpose();

    //landmark cone colour
    landmark.colour_ = colour;

    //add landmark to particle landmark list
    if (landmarks_.size() < num_lms)
        landmarks_.push_back(landmark);

    return landmark.mu_;
}

/**
 * @brief Calculates weight of particle given a landmark and an observation
 * @param innov_mean Eigen 2x1 innovation mean vector of observed landmark
 * @param lm_sigma Eigen 2x2 covariance matrix of the landmark
 * @param Hl Eigen 2x2 Jacobian matrix with respect to the landmark location
 * @param Hv Eigen 2x3 Jacobian matrix with respect to the robot location
 * @param R Eigen 2x2 covariance matrix of measurement noise
 * @return The calculated weight
 */
float Particle::calculateWeight(const Eigen::Vector2f& innov_mean, const Eigen::Matrix2f& lm_sigma, const Eigen::Matrix2f& Hl, const Eigen::Matrix<float, 2, 3>& Hv, const Eigen::Matrix2f& R)
{
    //calculates the weight of the particle with the current landmark and observation
    Eigen::Matrix2f L;
    L << (Hv * sigma_ * Hv.transpose()) + (Hl * lm_sigma * Hl.transpose()) + R;
    float numerator = expf(-0.5 * innov_mean.transpose() * L.inverse() * innov_mean);
    float denominator =  1 /  sqrtf(2.0 * M_PI * L.determinant());
    float weight = numerator / denominator;
    return weight;
}

/**
 * @brief Updates the state and covariance of the landmark
 * @param lm A Landmark object
 * @param H Jacobian matrix of landmark observation
 * @param innov_cov Eigen 2x2 innovation covariance matrix of observed landmark
 * @param innov_mean Eigen 2x1 innovation mean vector of observed landmark
 */
void Particle::updateLandmarkWithCholesky(Landmark& lm, const Eigen::Matrix2f& H, const Eigen::Matrix2f& innov_cov, const Eigen::Vector2f& innov_mean)
{
    //the Cholesky factorisation is used because it is more numerically stable than a naive implementation
    //it is usually used to avoid performing the inverse of a matrix, as it is a very costly operation 

    //make the matrix symmetric
    Eigen::Matrix2f S((innov_cov + innov_cov.transpose()) * 0.5);

    //calculate the L in the A=L(L.T) cholesky decomposition and calculate its inverse
    Eigen::Matrix2f L(S.llt().matrixL());
    Eigen::Matrix2f L_inv(L.inverse());

    //update the mean and covariance of the landmark
    Eigen::Matrix2f K1(lm.sigma_ * H.transpose() * L_inv);
    Eigen::Matrix2f K(K1 * L_inv.transpose());
    lm.mu_ += (K * innov_mean);
    lm.sigma_ -= (K1 * K1.transpose());
}


/**
 * @brief Checks if two landmarks are the same based on their colour and euclidean distance between them
 * @param lm1 First Landmark object
 * @param lm2 Second Landmark object
 * @return True if any stored landmark has the same colour and distance less than 20cm from lm
 */
bool Particle::sameLandmark(const Landmark& lm1, const Landmark& lm2)
{
    return ((lm1.colour_ == lm2.colour_) && (lm1.mu_ - lm2.mu_).norm() <= 0.20);
}

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
float Particle::dataAssociations(const Eigen::Matrix2f& lm_innov_cov, const Eigen::Vector2f& lm_innov_mean, const Eigen::Matrix<float, 2, 3>& Hv, const Eigen::Vector2f& zt, const perception_pkg::Cone& z, const Landmark& lm, const Eigen::Matrix2f& R)
{
    //Cholesky decomposition of the covariance, which is by definition definite and symmetric
    Eigen::Matrix3f cholesky_L = sigma_.llt().matrixL();

    //uniform distribution 
    boost::random::uniform_real_distribution<float> distribution(0.0, 1.0);

    //add sample taken from updated distribution to particle
    boost::shared_ptr<geometry_msgs::Point> z_ptr(new geometry_msgs::Point(z.location));
    Eigen::Vector2f measurement(getMeasurement(z_ptr));
    Innovation inn(computeInnovations(mu_, sigma_, lm, R));
    Eigen::Vector2f new_innov_mean(measurement - inn.predicted_observation);

    //after sampling pose from distribution, run the result to find data association value
    double numerator = exp(-0.5 * new_innov_mean.transpose() * lm_innov_cov.inverse() * new_innov_mean);
    double denominator = 1 / sqrtf(2 * M_PI * lm_innov_cov.determinant());
    float p = numerator / denominator;
    return p;
}

/**
 * @brief Generates a proposal distribution using the observations of the known features and samples a pose from it
 * @param known_features Vector of DataAssociation values corresponding to known features from newly observed cones 
 * @param R Eigen 2x2 covariance matrix of measurement noise
 */
void Particle::proposalSampling(const std::vector<DataAssociation> known_features, const Eigen::Matrix2f& R)
{
   //iterating through known features
    std::vector<DataAssociation>::const_iterator f;
    for(f = known_features.begin(); f != known_features.end(); ++f)
    {
        //recalculate innovations after landmark update
        Innovation new_innov = computeInnovations(mu_, sigma_, landmarks_.at(f->landmark_id), R);
        Eigen::Vector2f new_zt(new_innov.predicted_observation);
        Eigen::Vector2f measurement(f->measurement);
        Eigen::Vector2f new_innov_mean(measurement - new_zt);
        new_innov_mean(1) = angleWrap(new_innov_mean(1));
        Eigen::Matrix2f new_innov_cov(new_innov.innovation_covariance);
        Eigen::Matrix<float, 2, 3> new_Hv(new_innov.vehicle_jacobian);

        //incrementally refine proposal distribution
        proposalDistribution(new_innov_cov, new_innov_mean, new_Hv);
    }
    //Cholesky decomposition of the covariance, which is by definition definite and symmetric
    Eigen::Matrix3f cholesky_L = sigma_.llt().matrixL();

    //uniform distribution object
    boost::random::normal_distribution<float> distribution(0.0, 1.0);

    //get random sample from multivariate Gaussian/normal distribution
    Eigen::Vector3f uniform_randoms(distribution(rng), distribution(rng), distribution(rng));
    //std::cout << "uniform numbers\n" << uniform_randoms << std::endl;
    Eigen::Vector3f x_sample(cholesky_L * uniform_randoms * 0.1 + mu_);
    x_sample(2) = angleWrap(x_sample(2));

    mu_ = x_sample;
}

/**
 * @brief One step towards the generation of the proposal distribution using only one of the known feature data associations 
 * @param lm_innov_cov Eigen 2x2 innovation covariance matrix of observed landmark
 * @param lm_innov_mean Eigen 2x1 innovation mean vector of observed landmark
 * @param Hv Eigen 2x3 Jacobian matrix with respect to the robot location
 */
void Particle::proposalDistribution(const Eigen::Matrix2f& lm_innov_cov, const Eigen::Vector2f& lm_innov_mean, const Eigen::Matrix<float, 2, 3>& Hv)
{
   //proposal distribution covariance and mean
    sigma_ = (Hv.transpose() * lm_innov_cov.inverse() * Hv + sigma_.inverse()).inverse();

    mu_ += sigma_ * Hv.transpose() * lm_innov_cov.inverse() * lm_innov_mean;
    mu_(2) = angleWrap(mu_(2));
}