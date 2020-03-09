#include "test_slam_utils.hpp"
#include "testingparticle.hpp"
#include "testingfastslam.hpp"
#include <Eigen/Dense>
#include <Eigen/Cholesky>
#include <cmath>
#include <vector>
#include <Eigen/Core>
#include <algorithm>
#include <string>
#include "ros/ros.h"
#include <iostream>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <nav_msgs/Path.h>
#include <stdint.h>
#include <ctime>
#include <fstream>


std::time_t now = std::time(0);
boost::random::mt19937 rng(static_cast<uint32_t>(now));

/**
 * @brief Initialise a StateEstimation object and the Particles objects
 */
void TestStateEstimation::initialise()
{
    //load parameters from ROS Param Server
    max_speed_ = 10.0;
    max_steering_ = 0.523599;
    wheel_base_ = 1.55;
    wheel_diameter_ = 0.505;
    float sigmaV = 1.0;
    float sigmaG = 2.5 * M_PI / 180;
    float sigmaR = 0.5;
    float sigmaB = 2.5 * M_PI / 180;
    float resampling_ratio_ = num_particles_ * 0.75;

    //initialise control noise covariance matrix
    Q_ << powf(sigmaV, 2), 0, 0, powf(sigmaG, 2);

    //initialise measurement noise covariance matrix
    R_ << powf(sigmaR, 2), 0, 0, powf(sigmaB, 2);

    lap_closure_detected_ = false;

    slam_path_pub_ = nh_.advertise<nav_msgs::Path>("slam_path", 1000);
    motion_path_pub_ = nh_.advertise<nav_msgs::Path>("motion_path", 1000);
    particles_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("particles", 1);
    features_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("features", 1);

}

/**
 * @brief Normalizes the particle weights
 */
void TestStateEstimation::normaliseWeights()
{
    //sum weights of all particles 
    double sum_w = 0.0;
    std::vector<TestParticle>::iterator p;
    ROS_INFO("normalizing weights");
    for(p = particles_.begin(); p != particles_.end(); ++p)
            sum_w += p->weight_;
    std::cout << "sum: " << sum_w << std::endl;
    
    //normalize particle weights using sum of all weights
    try 
    {
        for(p = particles_.begin(); p != particles_.end(); ++p)
            p->weight_ /= sum_w;
            std::cout << "normalized weight: " << p->weight_ << std::endl;
    } 
    //if division by zero occurs, normalize weights using number of particles variable 
    catch(std::runtime_error& e) 
    {
        for(p = particles_.begin(); p != particles_.end(); ++p)
            p->weight_ = 1.0 / num_particles_;
    }   
}

/**
 * @brief State estimation prediction based on odometric and robot control state data
 * @param current_pose A RobotPose object, describing the current pose estimate
 * @param speed The current linear speed of the robot
 * @param steer The current steering angle of the robot
 * @param wb The wheelbase of the differential-drive robot, the distance between the centres of the front and rear wheels
 * @param delta Time step from last known pose
 * @param Gx Eigen 3x3 Jacobian with respect to location
 * @param Gv Eigen 3x2 Jacobian matrix with respect to control
 * @param Q Eigen 2x2 covariance matrix of control noise
 */
void TestParticle::motionUpdate(const Eigen::Vector3f& current_pose, float speed, float steer, float wb, float delta, const Eigen::Matrix3f& Gx, const Eigen::Matrix<float, 3, 2>& Gu, const Eigen::Matrix2f& Q)
{
    //calculate the difference from the last known pose
    float delta_x = speed * cosf(current_pose(2)) * delta;
    float delta_y = speed * sinf(current_pose(2)) * delta;
    std::cout << "delta y " << delta_y << std::endl;
    float delta_th = (speed * delta * tanf(steer)) / wb;

    //predict the particle mean
    mu_(0) += delta_x;
    mu_(1) += delta_y;
    mu_(2) = angleWrap(mu_(2) + delta_th);

    //predict the particle covariance
    sigma_ = Gx * sigma_ * Gx.transpose() + Gu * Q * Gu.transpose();

    std::cout << "mean prediction\n" << mu_ << "\n\ncovariance prediction\n" << sigma_ << std::endl;
}

/**
 * @brief Sampling step: applying the motion model to every particle
 */
void TestStateEstimation::prediction(Eigen::Vector2f u)
{
    //current linear velocity and angular velocity
    float speed = u(0);
    float steering_angle = u(1);

    //timestep
    float dt = 0.5;

    //current yaw pose
    float current_yaw = pose_(2);

    //Jacobian with respect to location
    Eigen::Matrix3f Gx;
    Gx << 1, 0, -speed * dt * sinf(current_yaw),
        0, 1, speed * dt * cosf(current_yaw), 0, 0, 1;

    //Jacobian with respect to control
    Eigen::Matrix<float, 3, 2> Gu;
    Gu << dt * cosf(current_yaw), -speed * dt * sinf(current_yaw), dt * sinf(current_yaw),
        speed * dt * cosf(current_yaw), dt * tanf(steering_angle) / wheel_base_,
        speed * dt / (pow(cosf(steering_angle), 2) * wheel_base_);

    std::vector<TestParticle>::iterator p;
    std::cout << "number of particles is " << particles_.size() << std::endl;
    for (p = particles_.begin(); p != particles_.end(); ++p)
    {
        p->motionUpdate(xEst_, speed, steering_angle, wheel_base_, dt, Gx, Gu, Q_);

    }

    boost::random::uniform_real_distribution<float> distribution(0.0, 1.0);
    std::cout << "random uniform number " << distribution(rng) << " sqrtQ00 " << sqrt(Q_(0, 0)) << " sqrtQ11 " << Q_(1, 1) << std::endl;
    std::cout << "Q\n" << Q_ << std::endl;
    pose_(0) += speed * cosf(pose_(2)) * dt;  
    pose_(1) += speed * sinf(pose_(2)) * dt;  
    pose_(2) = angleWrap(pose_(2) + ((speed * dt * tanf(steering_angle)) / wheel_base_)); 
    std::cout << "pose:\n" << pose_ << std::endl;
}

/**
 * @brief Correction step: applying the measurement model to every particle
 * @param observations A list of Cone messages
 * @param slam_phase The phase (mapping or localization) the car is currently executing
 */
void TestStateEstimation::correction(const std::vector<ConeLocation>& observations, TEST_SLAM_PHASE slam_phase)
{
    std::vector<TestParticle>::iterator p;
    int num = 1;
    std::vector<ConeLocation>::const_iterator z;

    int obsnum = 1;
    for(z = observations.begin(); z != observations.end(); ++z)
    {
        std::cout << "OBSERVATION NUMBER " << obsnum << std::endl;
        int num = 1;
        for (p = particles_.begin(); p != particles_.end(); ++p)
        {
            std::cout << "number of particles " << particles_.size() << std::endl;
            std::cout << "PARTICLE NUMBER " << num << std::endl;

            //unknwon data association
            p->measurementUpdate(*z, pose_, R_, Q_, slam_phase);
            if ((p->unknown_features.size() + p->landmarks_.size()) > 6)
            {
                ROS_ERROR("error");
                //exit(0);
            }
            ++num;
        }
        ++obsnum;
    }

    std::cout << "\n\n\nOBSERVATIONS PROCESSED\n\n" << std::endl;
    num = 1;
    for (p = particles_.begin(); p != particles_.end(); ++p)
    {     
        if (p->known_features.size() != 0)   
        {
            std::cout << "PROPOSAL DISTRIBUTION AND PROPOSAL SAMPLING" << std::endl;
            p->proposalSampling(p->known_features, R_);

            std::cout << "new mu\n" <<p->mu_ << std::endl;

            std::vector<DataAssociation>::const_iterator k; 
            for (k = p->known_features.begin(); k != p->known_features.end(); ++k)
            {
                Innovation new_innovation(computeInnovations(p->mu_, p->sigma_, p->landmarks_.at(k->landmark_id), R_));
                Eigen::Matrix<float, 2, 3> new_Hv(new_innovation.vehicle_jacobian);
                Eigen::Matrix2f new_Hl(new_innovation.feature_jacobian);
                Eigen::Vector2f new_zt(new_innovation.predicted_observation);
                Eigen::Matrix2f new_lm_innov_cov(new_innovation.innovation_covariance);

                Eigen::Vector2f measurement(k->measurement);
                Eigen::Vector2f new_lm_innov_mean(measurement - new_zt);
                std::cout << "innovation mean\n" << new_lm_innov_mean << std::endl;
                new_lm_innov_mean(1) = angleWrap(new_lm_innov_mean(1));
                
                //for the known feature, calculate importance factor and update the weight of the particle
                float weight = p->calculateWeight(new_lm_innov_mean, p->landmarks_.at(k->landmark_id).sigma_, new_Hl, new_Hv, R_);
                std::cout << "particle weight: " << p->weight_ << std::endl;
                p->weight_ *= weight;

                //update landmark with Cholesky decomposition that is more computationally efficient
                p->updateLandmarkWithCholesky(p->landmarks_.at(k->landmark_id), new_Hl, new_lm_innov_cov, new_lm_innov_mean);
            }
            p->known_features.clear();
        }

        //iterating through unknown features
        std::vector<DataAssociation>::const_iterator u; 
        for (u = p->unknown_features.begin(); u != p->unknown_features.end(); ++u)
        {
            p->addNewLandmark(u->measurement, "unknown", R_);
        }
        p->unknown_features.clear();

        //PRINT STUFF OUT
        int num = 0;
        std::vector<TestLandmark>::const_iterator lm;
        for(lm = p->landmarks_.begin(); lm != p->landmarks_.end(); ++lm)
        {
            ++num;
            std::cout << "landmark number " << num << "=> x:" << (*lm).mu_(0) << "; y:" << (*lm).mu_(1) << std::endl;
        }
        std::cout << "robot position mean after measurement model\n" << p->mu_ <<
            "\nrobot position covariance after measurement model\n" << p->sigma_ <<std::endl;
        ++num;
    }
}

/**
 * @brief Perform correction step on a particle
 * @param observations List of Cone messages
 * @param current_pose A RobotPose object
 * @param slam_phase The phase (mapping or localization) the car is currently executing
 */
void TestParticle::measurementUpdate(const ConeLocation& z, const Eigen::Vector3f& current_pose, const Eigen::Matrix2f& R, const Eigen::Matrix2f& Q, TEST_SLAM_PHASE slam_phase)
{
    const float MAX_RANGE = 20.0;

    Eigen::Vector2f measurement;
    
    //retrieve the measurement from the sensor reading
    measurement << sqrtf(powf((z).x - current_pose(0), 2) + powf((z).y - current_pose(1), 2)), 
        angleWrap(atan2f((z).y - current_pose(1), (z).x - current_pose(0)) - current_pose(2));

    //add some noise to the range observations
    boost::random::uniform_real_distribution<float> distribution(0.0, 1.0);

    //repeat until no NaN value
    /*float noisy_angle, noisy_range; 
    do {
        noisy_angle = angleWrap(measurement(1) + sqrtf(distribution(rng) * R(1, 1)));
    }
    while (noisy_angle != noisy_angle);

    do {
        noisy_range = measurement(0) + sqrtf(distribution(rng) * R(0, 0));
    }
    while (noisy_range != noisy_range);

    measurement(0) = noisy_range;
    measurement(1) = noisy_angle;
    */
    std::cout << "current mu: " << current_pose << std::endl;
    std::cout << "measurement: " << measurement << std::endl;
    std::cout << "z\n" << (z).x << ", " << (z).y << std::endl;

    if (measurement(0) > MAX_RANGE) {
        ROS_INFO("measurement is outside range of robot");
        return;
    }

    //assume the observations received are all unique and not repeated
    if(landmarks_.empty())
    {
        //add the first landmark to the particle
        addNewLandmark(measurement, "unknown", R);
        //PRINT STUFF OUT
        int num = 0;
        std::vector<TestLandmark>::const_iterator lm;
        for(lm = landmarks_.begin(); lm != landmarks_.end(); ++lm)
        {
            ++num;
            std::cout << "landmark number " << num << "=> x:" << (*lm).mu_(0) << "; y:" << (*lm).mu_(1) << std::endl;
        }
        std::cout << "robot position mean after measurement model\n" << mu_ <<
            "\nrobot position covariance after measurement model\n" <<sigma_ <<std::endl;
        return;
    }
    
    //data association variables
    Eigen::Matrix<float, 2, 3> best_Hv = Eigen::Matrix<float, 2, 3>::Zero();
    Eigen::Matrix2f best_Hl = Eigen::Matrix2f::Zero();
    Eigen::Matrix2f best_innov_cov = Eigen::Matrix2f::Zero();
    Eigen::Vector2f best_innov_mean = Eigen::Vector2f::Zero();
    float best_p = 0;
    int best_arg = 0;
    
    int i = 0;
    std::vector<TestLandmark>::const_iterator lm;
    for(lm = landmarks_.begin(); lm != landmarks_.end(); ++lm, ++i)
    {
        std::cout << "size of landmarks vector " << landmarks_.size() << std::endl;
        //data association: check if observation corresponds to a known or new map feature (landmark)
        Eigen::Vector2f map_feature((z).x, (z).y);
        std::cout << "difference b/w observation and landmark " << (map_feature - (*lm).mu_).norm() << std::endl;
        
        Innovation innovation(computeInnovations(mu_, sigma_, *lm, R));
        Eigen::Matrix<float, 2, 3> Hv(innovation.vehicle_jacobian);
        Eigen::Matrix2f Hl(innovation.feature_jacobian);
        Eigen::Vector2f zt(innovation.predicted_observation);
        Eigen::Matrix2f lm_innov_cov(innovation.innovation_covariance);
        

        Eigen::Vector2f lm_innov_mean(measurement - zt);
        lm_innov_mean(1) = angleWrap(lm_innov_mean(1));

        //perform data association to distinguish new features, and assigning correct observations to known features
        float p = dataAssociations(lm_innov_cov, lm_innov_mean, Hv, zt, z, *lm, R);
        std::cout << "ratio " << p << std::endl; 
        
        if ((i == 0) || (p > best_p))  
        {
            best_p = p;
            best_arg = i;
            best_Hv = Hv;
            best_innov_cov = lm_innov_cov;
            best_innov_mean = lm_innov_mean;
        }
    }

    ROS_WARN("BEST MATCH: landmark number %d", best_arg);
    std::cout << "best ratio value " << best_p << std::endl;   
    /*if (best_p == 0.0)
    {
        Eigen::Vector2f map_feature((z).x, (z).y);
        if ( (map_feature - (*lm).mu_).norm() < 0.5)
    } */
    Eigen::Vector2f map_feature((z).x, (z).y);
    if ((best_p < 0.00001) && ((map_feature - landmarks_.at(best_arg).mu_).norm() > 1.5))
    //PROBLEM WHEN THE RATIO IS 0, FIX IT 
    {
        DataAssociation d;
        d.measurement = measurement;
        d.landmark_id = best_arg;
        unknown_features.push_back(d);
    }
    else 
    {       
        DataAssociation d;
        d.measurement = measurement;
        d.landmark_id = best_arg;     
        known_features.push_back(d);
    }
    
    if (slam_phase == MAP_BUILDING) {
        ;//do stuff
    }
    else if (slam_phase == LOCALIZATION) {
        ;//do stuff
    }
    
    //PRINT STUFF OUT
    int num = 0;
    for(lm = landmarks_.begin(); lm != landmarks_.end(); ++lm)
    {
        ++num;
        std::cout << "landmark number " << num << "=> x:" << (*lm).mu_(0) << "; y:" << (*lm).mu_(1) << std::endl;
    }
    std::cout << "robot position mean after measurement model\n" << mu_ <<
        "\nrobot position covariance after measurement model\n" <<sigma_ <<std::endl;
}



/**
 * @brief Adds new landmark to the list of landmarks associated to the particle
 * @param ob A 2D vector describing the euclidean distance and yaw angle to the landmark
 * @param colour The colour of the cone landmark
 * @param R Eigen 2x2 covariance matrix of measurement noise
 */
void TestParticle::addNewLandmark(const Eigen::Vector2f& ob, const std::string& colour, const Eigen::Matrix2f& R)
{
    TestLandmark landmark;
    //std::cout << "adding landmark current mu\n" << mu_ << std::endl;
    //sine and cosine of yaw angle
    //WRONG NO NEED FOR ANGLE WRAP !!!!!!!!!!!
    float s = sinf(angleWrap(mu_(2) + ob(1)));
    float c = cosf(angleWrap(mu_(2) + ob(1)));

    std::cout << "mu " << mu_ << " // ob " << ob << std::endl;

    //x and y values of landmark respect to robot pose (maybe do it in map frame)
    float lm_x = mu_(0) + ob(0)*c;
    float lm_y = mu_(1) + ob(0)*s;

    //landmark state
    landmark.mu_ << lm_x, lm_y;

    //std::cout <<"inside addnewlandmark " << landmark.mu_ << std::endl;

    //landmark covariance
    Eigen::Matrix2f G;
    G << c, -ob(0) * s, s, ob(0) * c;
    landmark.sigma_  << G * R * G.transpose();
    //landmark.sigma_ << Eigen::Matrix2f::Identity();

    //landmark cone colour
    landmark.colour_ = colour;

    //add landmark to particle landmark list
    if (landmarks_.size() < num_lms)
    {
        landmarks_.push_back(landmark);
    }
    ROS_WARN("New landmark added");
    std::cout << landmark.mu_ << std::endl;
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
float TestParticle::calculateWeight(const Eigen::Vector2f& innov_mean, const Eigen::Matrix2f& lm_sigma, const Eigen::Matrix2f& Hl, const Eigen::Matrix<float, 2, 3>& Hv, const Eigen::Matrix2f& R)
{
    //calculates the weight of the particle with the current landmark and observation
    Eigen::Matrix2f L;
    L << (Hv * sigma_ * Hv.transpose()) + (Hl * lm_sigma * Hl.transpose()) + R;
    std::cout << "L:\n " << L << std::endl; 
    std::cout << innov_mean.transpose() * L.inverse() * innov_mean << std::endl;
    float numerator = expf(-0.5 * innov_mean.transpose() * L.inverse() * innov_mean);
    float denominator =  1 /  sqrtf(2.0 * M_PI * L.determinant());
    std::cout << "numerator: " << numerator << "\ndenominator: " << denominator << std::endl; 
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
void TestParticle::updateLandmark(TestLandmark& lm, const Eigen::Matrix2f& H, const Eigen::Matrix2f& innov_cov, const Eigen::Vector2f& innov_mean)
{
    //Calculate the EKF update given the prior state, the innovation and the linearised observation model
    Eigen::Matrix2f K(lm.sigma_ * H.transpose() * innov_cov.inverse());

    //update the mean and covariance of the landmark
    lm.mu_ += (K * innov_mean);
    lm.sigma_ *= (Eigen::Matrix2f::Identity() - (K * H));
}

/**
 * @brief Updates the state and covariance of the landmark
 * @param lm A Landmark object
 * @param H Jacobian matrix of landmark observation
 * @param innov_cov Eigen 2x2 innovation covariance matrix of observed landmark
 * @param innov_mean Eigen 2x1 innovation mean vector of observed landmark
 */
void TestParticle::updateLandmarkWithCholesky(TestLandmark& lm, const Eigen::Matrix2f& H, const Eigen::Matrix2f& innov_cov, const Eigen::Vector2f& innov_mean)
{
    //Calculate the EKF update given the prior state, the innovation and the linearised observation model
    //the Cholesky factorisation is used because more numerically stable than a naive implementation

    //make the matrix symmetric
    Eigen::Matrix2f S((innov_cov + innov_cov.transpose()) * 0.5);
    //std::cout << "innov_cov" << innov_cov << "\ns" << S <<std::endl;
    //calculate the L in the A=L(L.T) cholesky decomposition and calculate its inverse
    Eigen::Matrix2f L(S.llt().matrixL());
    Eigen::Matrix2f L_inv(L.inverse());
    //std::cout << "L" << L << "\nLinv" << L_inv <<std::endl;

    //update the mean and covariance of the landmark
    Eigen::Matrix2f K1(lm.sigma_ * H.transpose() * L_inv);
    Eigen::Matrix2f K(K1 * L_inv.transpose());
    lm.mu_ += (K * innov_mean);
    lm.sigma_ -= (K1 * K1.transpose());
    //std::cout <<"cholesky update=>\nmu:" << lm.mu_ << "\nsigma:" << lm.sigma_ << std::endl;
}



/**
 * @brief Checks if current landmark is the same as a previously observed landmark
 * @param lm A Landmark object
 * @param landmarks_to_check List of Landmark objects to check against
 * @return True if any stored landmark has the same colour and distance less than 20cm from lm
 */
bool TestParticle::sameLandmark(const TestLandmark& lm, const std::vector<TestLandmark>& landmarks_to_check)
{
    std::vector<TestLandmark>::const_iterator landmark;
    for(landmark = landmarks_to_check.begin(); landmark != landmarks_to_check.end(); ++landmark)
    {
        //could end up checking against itself-> fix this
        if (((*landmark).colour_ == lm.colour_) && ((*landmark).mu_ - lm.mu_).norm() <= 0.20)
            return true;
    }
    return false;
}

/**
 * 
 */
float TestParticle::dataAssociations(const Eigen::Matrix2f& lm_innov_cov, const Eigen::Vector2f& lm_innov_mean, const Eigen::Matrix<float, 2, 3>& Hv, const Eigen::Vector2f& zt, const ConeLocation& ob, const TestLandmark& lm, const Eigen::Matrix2f& R)
{
    //Cholesky decomposition of the covariance, which is by definition definite and symmetric
    Eigen::Matrix3f cholesky_L = sigma_.llt().matrixL();

    //uniform distribution object 

    //BETWEEN 0 AND 1 OR -1 AND 1 ????
    boost::random::uniform_real_distribution<float> distribution(0.0, 1.0);

    //add sample taken from updated distribution to particle
    Eigen::Vector2f measurement;
    measurement << sqrtf(powf(ob.x - mu_(0), 2) + powf(ob.y - mu_(1), 2)), 
        angleWrap(atan2f(ob.y - mu_(1), ob.x - mu_(0)) - mu_(2));
    Innovation inn(computeInnovations(mu_, sigma_, lm, R));
    std::cout << "measurement " << measurement << "// z " << inn.predicted_observation << std::endl;
    Eigen::Vector2f new_innov_mean(measurement - inn.predicted_observation);
    //std::cout << "old innov mean " << lm_innov_mean << " / new innov mean " << new_innov_mean << " / lm innov cov " << lm_innov_cov << std::endl;

    //after sampling pose from distribution, run the result to find data association value
    //std::cout << 0.5 * new_innov_mean.transpose() * lm_innov_cov.inverse() * new_innov_mean << std::endl;
    double numerator = exp(-0.5 * new_innov_mean.transpose() * lm_innov_cov.inverse() * new_innov_mean);
    //if (numerator < 0.000001)
     //   numerator = 0.00000001;
    double denominator = 1 / sqrtf(2 * M_PI * lm_innov_cov.determinant());
    std::cout << "numerator " << double(numerator) << " denominator " << double(denominator) << std::endl;
    float p = numerator / denominator;
    return p;
}

/**
 * @brief Generates a proposal distribution using an observation and samples a pose from it
 * @param lm_innov_cov Eigen 2x2 innovation covariance matrix of observed landmark
 * @param lm_innov_mean Eigen 2x1 innovation mean vector of observed landmark
 * @param Hv Eigen 2x3 Jaocbian matrix with respect to robot location
 * @param zt Eigen 2x1 measurement vector from robot pose to landmark
 * @return Data association value
 */
void TestParticle::proposalSampling(const std::vector<DataAssociation> known_features, const Eigen::Matrix2f& R)
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

    std::cout << "proposal sigma\n" << sigma_ << "\n\nproposal mu\n" << mu_ << std::endl; 

    //Cholesky decomposition of the covariance, which is by definition definite and symmetric
    Eigen::Matrix3f cholesky_L = sigma_.llt().matrixL();

    //uniform distribution object
    boost::random::normal_distribution<float> distribution(0.0, 1.0);

    //get random sample from multivariate Gaussian/normal distribution
    Eigen::Vector3f uniform_randoms(distribution(rng), distribution(rng), distribution(rng));
    std::cout << "uniforms\n" << uniform_randoms << std::endl;
    std::cout << "choleskyL\n" << cholesky_L << std::endl; 
    Eigen::Vector3f x_sample(cholesky_L * uniform_randoms + mu_);
    x_sample(2) = angleWrap(x_sample(2));

    mu_ = x_sample;
}

/**
 * @brief
 * @param
 * @return
 */
void TestParticle::proposalDistribution(const Eigen::Matrix2f& lm_innov_cov, const Eigen::Vector2f& lm_innov_mean, const Eigen::Matrix<float, 2, 3>& Hv)
{
   //proposal distribution covariance and mean
    sigma_ = (Hv.transpose() * lm_innov_cov.inverse() * Hv + sigma_.inverse()).inverse();

    //std::cout << "sigma after proposal sampling\n" << sigma_ << std::endl;
    //std::cout << "mu before proposal\n" << mu_ << std::endl;
    
    mu_ += sigma_ * Hv.transpose() * lm_innov_cov.inverse() * lm_innov_mean;
    mu_(2) = angleWrap(mu_(2));
    //std::cout << "mu before proposal\n" << mu_ << std::endl;

    //std::cout << "proposal added mu\n" << sigma_ * Hv.transpose() * lm_innov_cov.inverse() * lm_innov_mean << std::endl;
    //std::cout << "\nproposal mu\n" << mu_ << std::endl;
}

/**
 * @brief Resamples the particles based on the updated weights from the correction step
 */
void TestStateEstimation::resampling()
{
    //normalize weights
    normaliseWeights();

    //vector of weights
    Eigen::ArrayXf weights(num_particles_);
    int count = 0;
    std::vector<TestParticle>::const_iterator p;
    for (p = particles_.begin(); p != particles_.end(); ++p) {
        weights(count) = p->weight_;
        ++count;
    }

    //effective particle number and minimum number of particles
    float Neff = 1.0 / (weights.pow(2).sum());
    float Nmin = num_particles_ * resampling_ratio_;

    //vector for new set of particles
    std::vector<TestParticle> new_particles(num_particles_);

    //vector of cumulative weights
    Eigen::ArrayXf weights_cumulative(weights.size()), resample_base(weights.size()), resample_ids(weights.size());

    //uniform distribution object
    boost::random::uniform_real_distribution<float> distribution(0.0, 1.0);

    if (Neff < Nmin)
    {
        weights_cumulative << cumulativeSum(weights);
        //cumulative vector from 0 to [1 - (1 / NPARTICLES)] in steps of 1 / NPARTICLES
        resample_base << cumulativeSum(weights * 0.0 + 1 / num_particles_) - 1 / num_particles_;
        //add to every cumulative sum a number from uniform random distribution(0.0, 1.0 / NPARTICLES)
        //std::time_t now = std::time(0);
        //boost::random::mt19937 rng(static_cast<uint32_t>(now));
        resample_ids << resample_base + distribution(rng) / num_particles_;

        int count = 0;
        Eigen::VectorXd indexes(weights.size());
        for (int i = 0; i != num_particles_; ++i)
        {
            //see where each resample random number fits in the cumulative sum bins
            while ((count <= weights_cumulative.size() - 1) && (resample_ids(count) < weights_cumulative(i)))
                //if current random number is below a cumulative sum block, add the index of the weight block
                //if random number is bigger, pass to next cumulative sum by breaking out and increasing i
                //if array size of index number is exceeded, break out of while loop and increase i until break out of for loop
                indexes(i) = count;
                ++count;
        }

        //update particles with resampled particles
        std::vector<TestParticle> particles_copy(particles_);
        for (int i = 0; i != indexes.size(); ++i)
        {
            particles_.at(i).mu_ = particles_copy.at(indexes(i)).mu_;
            particles_.at(i).sigma_ = particles_copy.at(indexes(i)).sigma_;
            particles_.at(i).landmarks_ = particles_copy.at(indexes(i)).landmarks_;
            //reinitialise the weights so their sum adds up to 1
            particles_.at(i).weight_ = 1.0 / num_particles_;
        }
    }
}

/**
 * @brief Updates the SLAM estimate with the re-sampled particles and updated weights
 * @return A 3D Eigen Vector representing the final SLAM estimate for the current iteration
 */
Eigen::Vector3f TestStateEstimation::calculateFinalEstimate()
{
    //set SLAM estimate to zero and normalise all particle weights
    xEst_ = Eigen::Vector3f::Zero();
    normaliseWeights();

    //since all weights add up to 1, add from 0 the different weighted components of the state of each particle
    std::vector<TestParticle>::const_iterator p;
    for (p = particles_.begin(); p != particles_.end(); ++p) 
    {
        std::cout << "particle state\n" << p->mu_ << std::endl;
        std::cout << p->weight_ << std::endl;
        xEst_(0) += p->weight_ * p->mu_(0);
        xEst_(1) += p->weight_ * p->mu_(1);
        xEst_(2) = angleWrap(xEst_(2) + p->weight_ * p->mu_(2));
    }
    std::cout << "dead reckoning state\n" << pose_ << std::endl;
    //SHOULD THE ODOMETRIC POSE BE UPDATED WITH THE XEST FINAL ITERATION VALUE ????
    return xEst_;
}

int main(int argc, char** argv)
{
    ROS_INFO("HELLO");
    ros::init(argc, argv, "slam_node");
    ros::NodeHandle nh("~");
    ROS_INFO("HELLO");

    std::vector<ConeLocation> cone_locations;
    /*
    //create input filestream
    std::string file_name;
    nh.getParam("csv_map_path", file_name);
    std::ifstream file(file_name.c_str());

    //make sure the file is open
    if (!file.is_open()) throw std::runtime_error("Could not open file");

    //helpers
    std::vector<ConeLocation> locations;
    std::string line;

    //skip first line
    std::getline(file, line);

    //store all x and y coordinates in following lines
    while (std::getline(file, line))
    {
        std::istringstream ss(line);
        std::string token;
        std::vector<std::string> row;

        while (std::getline(ss, token, ','))
        {
            //process each token
            row.push_back(token);
        }

        //check that they are cones are not other objects from the csv file 
        if (((row[0].compare("yellow"))==0) || ((row[0].compare("blue"))==0))
        {
            ConeLocation cone;
            cone.x = std::atof(row[1].c_str());
            cone.y = std::atof(row[2].c_str());
           cone_locations.push_back(cone);

        }
    }
    */
	
	ConeLocation c1 = {10.0, -2.0, 3.0};
	ConeLocation c2 = {15.0, 10.0, 3.0};
	ConeLocation c3= {15.0, 15.0, 3.0};
	ConeLocation c4 = {10.0, 20.0, 3.0};
	ConeLocation c5 = {-5.0, 5.0, 3.0};
	ConeLocation c6 = {-10.0, 15.0, 3.0};
	cone_locations.push_back(c1);
	cone_locations.push_back(c2);
	cone_locations.push_back(c3);
	cone_locations.push_back(c4);
	cone_locations.push_back(c5);
	cone_locations.push_back(c6);

	double time = 0.0;

	//current robot pose based on odometry/dead reckoning
	Eigen::Vector3f xOdom;
	xOdom << 0.0, 0.0, 0.0;

	const float SIM_TIME = 15.0;

	//SLAM estimate
	Eigen::Vector3f xEst;
	xEst << 0.0, 0.0, 0.0;
	//history
	//Eigen::Array3Xf hxEst;

    //THIS WAY ------<>>>>>>>>>>>>
    Eigen::Matrix<float, 3, 2> testing;
    testing << 1, 2, 3, 2, 4, 5;

	//controls
	Eigen::Vector2f u;
	double v, yaw;

	//ADD NOISE TO THE SPOOFED MOTION MODEL SO YOU HAVE A TRUE XODOM AND ESTIMATED ONE
    TestStateEstimation fastslam(5, cone_locations.size(), 2);
    fastslam.initialise();
	while (SIM_TIME >= time)
	{
		time += 0.5;

		if (time < 1.0) {
			v = 0.0;
			yaw = 0.0;
		} else {
			v += 0.5;
			yaw = 1.0;
		}
		Eigen::Vector2f u;
		u << v, yaw;

        std::cout << "linear velocity:" << v << " angular velocity:" << yaw << std::endl;

        //when i declare an eigen with an X, I need to specify how much is that value 

		//insert fast slam 2 algorithm
		fastslam.prediction(u);
		fastslam.correction(cone_locations, MAP_BUILDING);
        //fastslam.resampling();
		xEst = fastslam.calculateFinalEstimate();
        std::cout << "State Estimation Final x:" << xEst(0) << " y:" << xEst(1) << " yaw: " << xEst(2) << std::endl;

		//store history data
		//hxEst(time/0.5 - 1) = xEst(0), xEst(1), xEst(2);

		//plot the landmarks
		std::vector<ConeLocation>::const_iterator iter;
		std::vector<float> x, y;
		for (iter = cone_locations.begin(); iter != cone_locations.end(); ++iter)
		{
			//x.push_back((*iter).x);
			//y.push_back((*iter).y);
            std::cout << "x: "  << (*iter).x  << " ; y " << (*iter).y << std::endl;
		}
		//matplotlibcpp::figure();
		//matplotlibcpp::plot(x, y);

	}
	return 0;
}

