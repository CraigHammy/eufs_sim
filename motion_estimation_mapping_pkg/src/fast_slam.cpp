#include "slam_utils.hpp"
#include "particle.hpp"
#include "fast_slam.hpp"
#include <Eigen/Dense>
#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <nav_msgs/Odometry.h>
#include <ackermann_msgs/AckermannDriveStamped.h>
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

std::time_t now = std::time(0);
boost::random::mt19937 rng(static_cast<uint32_t>(now));

/**
 * @brief Initialise a StateEstimation object and the Particles objects
 */
void StateEstimation::initialise()
{
    //control and measurement noise covariance matrix coefficients 
    float sigmaV, sigmaG;
    float sigmaR, sigmaB;

    //load parameters from ROS Param Server
    nh_.param("max_speed", max_speed_);
    nh_.param("max_steering", max_steering_);
    nh_.param("wheel_base", wheel_base_);
    nh_.param("wheel_diameter", wheel_diameter_);
    nh_.param("control_noise_velocity", sigmaV, float(1.0));
    nh_.param("control_noise_steering_angle", sigmaG, float(2.5 * M_PI / 180));
    nh_.param("measurement_noise_euclidean_distance", sigmaR, float(0.5));
    nh_.param("measurement_noise_angle_difference", sigmaB, float(2.5 * M_PI / 180));
    nh_.param("resampling_ratio_particles", resampling_ratio_, float(num_particles_ * 0.75));

    //get joint state values to retrieve current linear speed and steering angle of the car 
    while (!read_state_) 
        ;
    std::vector<std::string> joint_names = joint_state_.name;
    frw_pos_ = find(joint_names.begin(), joint_names.end(), std::string("right_front_axle")) - joint_names.begin();
    flw_pos_ = find(joint_names.begin(), joint_names.end(), std::string("left_front_axle")) - joint_names.begin();
    brw_vel_ = find (joint_names.begin(),joint_names.end(), std::string("right_rear_axle")) - joint_names.begin();
    blw_vel_ = find (joint_names.begin(),joint_names.end(), std::string("left_rear_axle")) - joint_names.begin();
    
    //initialise control noise covariance matrix 
    Q_ << powf(sigmaV, 2), 0, 0, powf(sigmaG, 2);

    //initialise measurement noise covariance matrix
    R_ << powf(sigmaR, 2), 0, 0, powf(sigmaB, 2);

    lap_closure_detected_ = false;
    read_state_ = false;
}

/**
 * @brief Runs the FastSLAM2.0 algorithm
 * @param observations List of Cone messages 
 */
void StateEstimation::FastSLAM2(const std::vector<perception_pkg::Cone>& observations)
{
    while(1)
    {
        prediction();
        if (!lap_closure_detected_)
        {
            correction(observations, MAP_BUILDING);
        }
        else 
        {
            correction(observations, LOCALIZATION);
        }
        calculateFinalEstimate();
    }
}

/**
 * @brief Normalizes the particle weights
 */
void StateEstimation::normaliseWeights()
{   
    //sum weights of all particles 
    double sum_w = 0.0;
    std::vector<Particle>::iterator p;
    for(p = particles_.begin(); p != particles_.end(); ++p)
            sum_w += p->weight_;
    
    //normalize particle weights using sum of all weights
    try 
    {
        for(p = particles_.begin(); p != particles_.end(); ++p)
            p->weight_ /= sum_w;
    } 
    //if division by zero occurs, normalize weights using number of particles variable 
    catch(std::runtime_error& e) 
    {
        for(p = particles_.begin(); p != particles_.end(); ++p)
            p->weight_ = 1.0 / num_particles_;
    }   
}

/**
 * @brief Callback for receiving odometry data
 * @param msg An Odometry message
 */
void StateEstimation::odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    //odometry pose 
    pose_(0) = msg->pose.pose.position.x;
    pose_(1) = msg->pose.pose.position.y;
    pose_(2) = msg->pose.pose.orientation.z;
}

/**
 * @brief Callback for receiving command data
 * @param msg An AckermannDriveStamped message
 */
void StateEstimation::commandCallback(const ackermann_msgs::AckermannDriveStamped::ConstPtr& msg)
{
    //motion commands
    u_.speed = msg->drive.speed;
    u_.steering_angle = msg->drive.steering_angle;
}

/**
 * @brief Callback for receiving joint state data
 * @param msg A JointState message
 */
void StateEstimation::jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
    //joint states values
    joint_state_ = *msg;
    read_state_ = true;
}

/**
 * @brief Callback for receiving detected Cone data
 * @param msg A Cone message
 */
void StateEstimation::coneCallback(const perception_pkg::Cone::ConstPtr& msg)
{
    if (msg->colour.compare("orange") && sqrtf(powf(msg->location.x, 2) + powf(msg->location.y, 2)) < 1.0)
        lap_closure_detected_ = true;
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
void Particle::motionUpdate(const Eigen::Vector3f& current_pose, float speed, float steer, float wb, float delta, const Eigen::Matrix3f& Gx, const Eigen::Matrix<float, 3, 2>& Gu, const Eigen::Matrix2f& Q)
{
    //calculate the difference from the last known pose
    float delta_x = speed * cosf(current_pose(2)) * delta;
    float delta_y = speed * sinf(current_pose(2)) * delta;
    float delta_th = (speed * delta * tanf(steer)) / wb;

    //predict the particle mean 
    mu_(0) += delta_x;
    mu_(1) += delta_y;
    mu_(2) = angleWrap(mu_(2) + delta_th);

    //predict the particle covariance
    sigma_ = Gx * sigma_ * Gx.transpose() + Gu * Q * Gu.transpose();
}

/**
 * @brief Sampling step: applying the motion model to every particle
 */
void StateEstimation::prediction()
{
    //current robot steering angle from joint state values
    float steering_angle = (joint_state_.position[frw_pos_] + joint_state_.position[flw_pos_]) / 2.0;

    //retrieving linear velocity and capping it using min and max achievable values
    float reference_speed; 
    reference_speed = (u_.speed > max_speed_) ? max_speed_ : u_.speed;
    reference_speed = (u_.speed < -max_speed_) ? -max_speed_ : u_.speed;

    //current robot linear velocity from joint state values
    float v1, v2, speed;
    if (reference_speed == 0.0) 
    {
        v1 = 0.0;
        v2 = 0.0;
    } 
    else 
    {
        v1 = joint_state_.velocity[blw_vel_] * (wheel_diameter_ / 2.0);
        v2 = joint_state_.velocity[brw_vel_] * (wheel_diameter_ / 2.0);
    }
    speed = -(v1 + v2) / 2.0;

    //timestep
    float dt = 1.0 / update_frequency_;

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
    
    std::vector<Particle>::iterator p;
    for (p = particles_.begin(); p != particles_.end(); ++p)
        p->motionUpdate(pose_, speed, steering_angle, wheel_base_, dt, Gx, Gu, Q_);
}

/**
 * @brief Correction step: applying the measurement model to every particle
 * @param observations A list of Cone messages 
 * @param slam_phase The phase (mapping or localization) the car is currently executing 
 */
void StateEstimation::correction(const std::vector<perception_pkg::Cone>& observations, SLAM_PHASE slam_phase)
{
    std::vector<Particle>::iterator p;
    std::vector<perception_pkg::Cone>::const_iterator z;

    int obsnum = 1;
    for(z = observations.begin(); z != observations.end(); ++z)
    {
        for (p = particles_.begin(); p != particles_.end(); ++p)
        {
            //unknwon data association
            p->measurementUpdate(*z, pose_, R_, Q_, slam_phase);
            if ((p->unknown_features_.size() + p->landmarks_.size()) > 6)
            {
                ROS_ERROR("error");
                //exit(0);
            }
        }
    }

    for (p = particles_.begin(); p != particles_.end(); ++p)
    {     
        if (p->known_features_.size() != 0)   
        {
            p->proposalSampling(p->known_features_, R_);

            std::vector<DataAssociation>::const_iterator k; 
            for (k = p->known_features_.begin(); k != p->known_features_.end(); ++k)
            {
                Innovation new_innovation(computeInnovations(p->mu_, p->sigma_, p->landmarks_.at(k->landmark_id), R_));
                Eigen::Matrix<float, 2, 3> new_Hv(new_innovation.vehicle_jacobian);
                Eigen::Matrix2f new_Hl(new_innovation.feature_jacobian);
                Eigen::Vector2f new_zt(new_innovation.predicted_observation);
                Eigen::Matrix2f new_lm_innov_cov(new_innovation.innovation_covariance);

                Eigen::Vector2f measurement(k->measurement);
                Eigen::Vector2f new_lm_innov_mean(measurement - new_zt);
                new_lm_innov_mean(1) = angleWrap(new_lm_innov_mean(1));
                
                //for the known feature, calculate importance factor and update the weight of the particle
                float weight = p->calculateWeight(new_lm_innov_mean, p->landmarks_.at(k->landmark_id).sigma_, new_Hl, new_Hv, R_);
                p->weight_ *= weight;

                //update landmark with Cholesky decomposition that is more computationally efficient
                p->updateLandmarkWithCholesky(p->landmarks_.at(k->landmark_id), new_Hl, new_lm_innov_cov, new_lm_innov_mean);
            }
            p->known_features_.clear();
        }

        //iterating through unknown features
        std::vector<DataAssociation>::const_iterator u; 
        for (u = p->unknown_features_.begin(); u != p->unknown_features_.end(); ++u)
        {
            p->addNewLandmark(u->measurement, u->colour, R_);
        }
        p->unknown_features_.clear();
    }
}

/**
 * @brief Perform correction step on a particle
 * @param observations List of Cone messages 
 * @param current_pose A RobotPose object 
 * @param slam_phase The phase (mapping or localization) the car is currently executing 
 */
void Particle::measurementUpdate(const perception_pkg::Cone& z, Eigen::Vector3f& current_pose, const Eigen::Matrix2f& R, const Eigen::Matrix2f& Q, SLAM_PHASE slam_phase)
{

    boost::shared_ptr<geometry_msgs::Point> z_ptr(new geometry_msgs::Point((z).location));
    Eigen::Vector2f measurement(getMeasurement(z_ptr));

    //add some noise to the range observations
    boost::random::uniform_real_distribution<float> distribution(0.0, 1.0);

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
    if ((best_p < 0.00001) && ((map_feature - landmarks_.at(best_arg).mu_).norm() > 1.5))
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
 * @param ob A 2D vector describing the euclidean distance and yaw angle to the landmark 
 * @param colour The colour of the cone landmark 
 * @param R Eigen 2x2 covariance matrix of measurement noise
 */
void Particle::addNewLandmark(const Eigen::Vector2f& ob, const std::string& colour, const Eigen::Matrix2f& R)
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
void Particle::updateLandmark(Landmark& lm, const Eigen::Matrix2f& H, const Eigen::Matrix2f& innov_cov, const Eigen::Vector2f& innov_mean)
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
void Particle::updateLandmarkWithCholesky(Landmark& lm, const Eigen::Matrix2f& H, const Eigen::Matrix2f& innov_cov, const Eigen::Vector2f& innov_mean)
{
    //Calculate the EKF update given the prior state, the innovation and the linearised observation model
    //the Cholesky factorisation is used because more numerically stable than a naive implementation

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
 * @brief Checks if current landmark is the same as a previously observed landmark
 * @param lm A Landmark object
 * @param landmarks_to_check List of Landmark objects to check against 
 * @return True if any stored landmark has the same colour and distance less than 20cm from lm
 */
bool Particle::sameLandmark(const Landmark& lm, const std::vector<Landmark>& landmarks_to_check)
{
    std::vector<Landmark>::const_iterator landmark;
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
float Particle::dataAssociations(const Eigen::Matrix2f& lm_innov_cov, const Eigen::Vector2f& lm_innov_mean, const Eigen::Matrix<float, 2, 3>& Hv, const Eigen::Vector2f& zt, const perception_pkg::Cone& ob, const Landmark& lm, const Eigen::Matrix2f& R)
{
    //Cholesky decomposition of the covariance, which is by definition definite and symmetric
    Eigen::Matrix3f cholesky_L = sigma_.llt().matrixL();

    //uniform distribution object 

    //BETWEEN 0 AND 1 OR -1 AND 1 ????
    boost::random::uniform_real_distribution<float> distribution(0.0, 1.0);

    //add sample taken from updated distribution to particle
    boost::shared_ptr<geometry_msgs::Point> z_ptr(new geometry_msgs::Point(ob.location));
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
 * @brief Generates a proposal distribution using an observation and samples a pose from it
 * @param lm_innov_cov Eigen 2x2 innovation covariance matrix of observed landmark
 * @param lm_innov_mean Eigen 2x1 innovation mean vector of observed landmark
 * @param Hv Eigen 2x3 Jaocbian matrix with respect to robot location
 * @param zt Eigen 2x1 measurement vector from robot pose to landmark 
 * @return Data association value
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
    Eigen::Vector3f x_sample(cholesky_L * uniform_randoms + mu_);
    x_sample(2) = angleWrap(x_sample(2));

    mu_ = x_sample;
}

/**
 * @brief
 * @param
 * @return
 */
void Particle::proposalDistribution(const Eigen::Matrix2f& lm_innov_cov, const Eigen::Vector2f& lm_innov_mean, const Eigen::Matrix<float, 2, 3>& Hv)
{
   //proposal distribution covariance and mean
    sigma_ = (Hv.transpose() * lm_innov_cov.inverse() * Hv + sigma_.inverse()).inverse();
    
    mu_ += sigma_ * Hv.transpose() * lm_innov_cov.inverse() * lm_innov_mean;
    mu_(2) = angleWrap(mu_(2));
}

/**
 * @brief Resamples the particles based on the updated weights from the correction step
 */
void StateEstimation::resampling()
{
    //normalize weights
    normaliseWeights();
    
    //vector of weights
    Eigen::ArrayXf weights(num_particles_);
    int count = 0;
    std::vector<Particle>::const_iterator p;
    for (p = particles_.begin(); p != particles_.end(); ++p) {
        weights(count) = (*p).weight_;
        ++count;
    }

    //effective particle number and minimum number of particles
    float Neff = 1.0 / (weights.pow(2).sum());
    float Nmin = num_particles_ * resampling_ratio_;

    //vector for new set of particles
    std::vector<Particle> new_particles(num_particles_);

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
        std::vector<Particle> particles_copy(particles_);
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
Eigen::Vector3f StateEstimation::calculateFinalEstimate()
{
    //set SLAM estimate to zero and normalise all particle weights 
    xEst_ = Eigen::Vector3f::Zero();
    normaliseWeights();

    //since all weights add up to 1, add from 0 the different weighted components of the state of each particle 
    std::vector<Particle>::const_iterator p;
    for (p = particles_.begin(); p != particles_.end(); ++p) 
    {
        xEst_(0) += p->weight_ * p->mu_(0);
        xEst_(1) += p->weight_ * p->mu_(1);
        xEst_(2) = angleWrap(xEst_(2) + p->weight_ * p->mu_(2));
    }
    return xEst_;
}

int main(int argc, char** argv)
{
    return 0;
}

