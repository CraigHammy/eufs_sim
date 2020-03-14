#include "slam_utils.hpp"
#include "particle.hpp"
#include "fast_slam.hpp"
#include <Eigen/Dense>
#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <sensor_msgs/JointState.h>
#include <nav_msgs/Odometry.h>
#include <ackermann_msgs/AckermannDriveStamped.h>
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
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <motion_estimation_mapping_pkg/FastSlamAction.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <sensor_msgs/ChannelFloat32.h>

std::time_t now = std::time(0);
boost::random::mt19937 rng(static_cast<uint32_t>(now));

/**
 * @brief Initialise a StateEstimation object and the Particles objects
 */
void StateEstimation::initialise()
{
    //read_state_ = false;
    start_ = false;

    initialiseSubscribers();
    initialisePublishers();

    current_time_ = ros::Time::now();
    last_time_ = current_time_;

    //control and measurement noise covariance matrix coefficients
    float sigmaV, sigmaG;
    float sigmaR, sigmaB;

    //load parameters from ROS Param Server
    private_nh_.getParam("max_speed", max_speed_);
    private_nh_.getParam("max_steering", max_steering_);
    private_nh_.getParam("wheel_base", wheel_base_);
    private_nh_.getParam("wheel_diameter", wheel_diameter_);
    nh_.param("control_noise_velocity", sigmaV, float(1.0));
    nh_.param("control_noise_steering_angle", sigmaG, float(2.5 * M_PI / 180));
    nh_.param("measurement_noise_euclidean_distance", sigmaR, float(0.5));
    nh_.param("measurement_noise_angle_difference", sigmaB, float(2.5 * M_PI / 180));
    nh_.param("resampling_ratio_particles", resampling_ratio_, float(num_particles_ * 0.75));

    //get joint state values to retrieve current linear speed and steering angle of the car
    /*while (!read_state_)
       ;

    std::vector<std::string> joint_names = joint_state_.name;
    frw_pos_ = find(joint_names.begin(), joint_names.end(), std::string("right_front_axle")) - joint_names.begin();
    flw_pos_ = find(joint_names.begin(), joint_names.end(), std::string("left_front_axle")) - joint_names.begin();
    brw_vel_ = find (joint_names.begin(),joint_names.end(), std::string("right_rear_axle")) - joint_names.begin();
    blw_vel_ = find (joint_names.begin(),joint_names.end(), std::string("left_rear_axle")) - joint_names.begin();
    */

    //initialise control noise covariance matrix
    Q_ << powf(sigmaV, 2), 0, 0, powf(sigmaG, 2);

    //initialise measurement noise covariance matrix
    R_ << powf(sigmaR, 2), 0, 0, powf(sigmaB, 2);

    lap_closure_detected_ = false;
    read_state_ = false;
    ROS_WARN("StateEstimation object initialised");
}

void StateEstimation::initialisePublishers()
{
    odom_path_pub_ = nh_.advertise<nav_msgs::Path>("odom_path", 1);
    pred_path_pub_ = nh_.advertise<nav_msgs::Path>("prediction_path", 1);
    slam_path_pub_ = nh_.advertise<nav_msgs::Path>("slam_path", 1);
    landmark_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/landmark_cloud", 1);
    slam_estimate_pub_ = nh_.advertise<nav_msgs::Odometry>("/slam_estimate", 1);
}

void StateEstimation::initialiseSubscribers()
{
    odom_sub_ = nh_.subscribe<nav_msgs::Odometry>("/ground_truth/odom", 1, boost::bind(&StateEstimation::odomCallback, this, _1));
    //joint_states_sub_ = nh_.subscribe<sensor_msgs::JointState>("/eufs/joint_states", 1, boost::bind(&StateEstimation::jointStateCallback, this, _1));
    wheel_speeds_sub_ = nh_.subscribe<eufs_msgs::WheelSpeedsStamped>("/ros_can/wheel_speeds", 1, boost::bind(&StateEstimation::wheelSpeedCallback, this, _1));
    //command_sub_ = nh_.subscribe<ackermann_msgs::AckermannDriveStamped>("/cmd_vel_out", 1, boost::bind(&StateEstimation::commandCallback, this, _1));
    cone_sub_ = nh_.subscribe<perception_pkg::Cone>("/cones", 1, boost::bind(&StateEstimation::coneCallback, this, _1));
}   
/**
 * @brief Runs the FastSLAM2.0 algorithm
 * @param observations List of Cone messages
 */
void StateEstimation::FastSLAM2()
{
    while(1)
    {
        prediction();
        if (!lap_closure_detected_)
        {
            correction(MAP_BUILDING);
        }
        else
        {
            correction(LOCALIZATION);
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
 * @brief Callback for receiving WheelSpeeds data
 * @param msg A WheelSpeeds message
 */
void StateEstimation::wheelSpeedCallback(const eufs_msgs::WheelSpeedsStamped::ConstPtr& msg)
{
    //ROS_ERROR("wheel speeds received");
    float rpm_rb = msg->rb_speed; 
    float rpm_lb = msg->lb_speed;
    float rpm_rf = msg->rf_speed;
    float rpm_lf = msg->lf_speed;
    //transform from RPM to m/s
    float right_back = 2 * M_PI * (wheel_diameter_/2) * rpm_rb / 60;
    float left_back = 2 * M_PI * (wheel_diameter_/2) * rpm_lb / 60;
    float right_front = 2 * M_PI * (wheel_diameter_/2) * rpm_rf / 60;
    float left_front = 2 * M_PI * (wheel_diameter_/2) * rpm_lf / 60;
    u_.speed = (right_back + left_back) / 2.0;
    u_.steering_angle = msg->steering;
    //start_ = true;

    //ROS_WARN("rpm : %f, wheel diameter: %f", msg->rb_speed, wheel_diameter_);
    //ROS_WARN("linear speed : %f, steering angle: %f", u_.speed, u_.steering_angle);
}

/**
 * @brief Callback for receiving odometry data
 * @param msg An Odometry message
 */
void StateEstimation::odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    //odometry pose
    //ROS_ERROR("odometry received");
    start_ = true;
    pose_(0) = msg->pose.pose.position.x;
    pose_(1) = msg->pose.pose.position.y;
    pose_(2) = msg->pose.pose.orientation.z;

    geometry_msgs::PoseStamped current_pose;
    current_pose.pose.position.x = pose_(0);
    current_pose.pose.position.y = pose_(1);
    current_pose.pose.position.z = 0;
    tf2::Quaternion orientation;
    orientation.setRPY(0, 0, pose_(2));

    current_pose.header.frame_id = "track";
    current_pose.header.stamp = ros::Time::now();
    
    odom_path_.header = current_pose.header;
    odom_path_.poses.push_back(current_pose);
    odom_path_pub_.publish(odom_path_);
}

/**
 * @brief Callback for receiving command data
 * @param msg An AckermannDriveStamped message
 */
void StateEstimation::commandCallback(const ackermann_msgs::AckermannDriveStamped::ConstPtr& msg)
{
    //motion commands
    //u_.speed = msg->drive.speed;
    //u_.steering_angle = msg->drive.steering_angle;
   // ROS_WARN("linear speed : %f, steering angle: %f", u_.speed, u_.steering_angle);
}

/**
 * @brief Callback for receiving joint state data
 * @param msg A JointState message
 */
void StateEstimation::jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
    //joint states values
    //ROS_ERROR("CALLBACK: Joint states published.");
    joint_state_ = *msg;
}

/**
 * @brief Callback for receiving detected Cone data
 * @param msg A Cone message
 */
void StateEstimation::coneCallback(const perception_pkg::Cone::ConstPtr& msg)
{
    if (msg->colour.compare("orange") && sqrtf(powf(msg->location.x, 2) + powf(msg->location.y, 2)) < 1.0)
        lap_closure_detected_ = true;
    input_data_.push_back(*msg);
    ROS_INFO("cone published");
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
    //mu_(0) += delta_x;
    //mu_(1) += delta_y;
    //mu_(2) += delta_th;

    mu_ = current_pose;

    //predict the particle covariance
    sigma_ = Gx * sigma_ * Gx.transpose() + Gu * Q * Gu.transpose();
}

/**
 * @brief Sampling step: applying the motion model to every particle
 */
void StateEstimation::prediction()
{
    //ROS_WARN("prediction");
    //current robot steering angle from joint state values
    /*float steering_angle = (joint_state_.position[frw_pos_] + joint_state_.position[flw_pos_]) / 2.0;

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
    */
   float steering_angle = u_.steering_angle;
   float speed = u_.speed;
   if (speed > max_speed_) speed = max_speed_;
   if (speed < -max_speed_) speed = -max_speed_;
   if (steering_angle > max_steering_) steering_angle = max_steering_;
   if (steering_angle < -max_steering_) steering_angle = -max_steering_;

    //timestep
    //float dt = 1.0 / update_frequency_;
    current_time_ = ros::Time::now();
    float dt = current_time_.toSec() - last_time_.toSec();
    //ROS_WARN("dt: %f", dt);
    last_time_ = current_time_;

    std::vector<Particle>::iterator p;
    for (p = particles_.begin(); p != particles_.end(); ++p)
    {
        //current yaw pose
        float current_yaw = p->mu_(2);

        //Jacobian with respect to location
        Eigen::Matrix3f Gx;
        Gx << 1, 0, -speed * dt * sinf(current_yaw),
            0, 1, speed * dt * cosf(current_yaw), 0, 0, 1;

        //Jacobian with respect to control
        Eigen::Matrix<float, 3, 2> Gu;
        Gu << dt * cosf(current_yaw), -speed * dt * sinf(current_yaw),
            dt * sinf(current_yaw), speed * dt * cosf(current_yaw),
            dt * tanf(steering_angle) / wheel_base_, speed * dt / (pow(cosf(steering_angle), 2) * wheel_base_);

        p->motionUpdate(pose_, speed, steering_angle, wheel_base_, dt, Gx, Gu, Q_);

        /*geometry_msgs::PoseStamped current_pose;
        current_pose.pose.position.x = p->mu_(0);
        current_pose.pose.position.y = p->mu_(1);
        current_pose.pose.position.z = 0.0;
        geometry_msgs::Quaternion orientation = tf::createQuaternionMsgFromYaw(p->mu_(2));
        current_pose.pose.orientation = orientation;
        //ROS_WARN("x: %f, y: %f, yaw: %f", p->mu_(0), p->mu_(1), p->mu_(2));

        current_pose.header.frame_id = "track";
        current_pose.header.stamp = ros::Time::now();
        
        pred_path_.header = current_pose.header;
        pred_path_.poses.push_back(current_pose);
        //ROS_INFO("hello1");
        pred_path_pub_.publish(pred_path_);*/
        //ROS_INFO("hello2");
    }
}

/**
 * @brief Correction step: applying the measurement model to every particle
 * @param observations A list of Cone messages
 * @param slam_phase The phase (mapping or localization) the car is currently executing
 */
void StateEstimation::correction(SLAM_PHASE slam_phase)
{
    //ROS_WARN("correction");
    std::vector<Particle>::iterator p;
    std::deque<perception_pkg::Cone>::const_iterator z;
    std::deque<perception_pkg::Cone> observations(input_data_.begin(), input_data_.end());
    int input_size = observations.size();

    for(z = observations.begin(); z != observations.end(); ++z)
    {
        for (p = particles_.begin(); p != particles_.end(); ++p)
        {
            //unknwon data association
            p->measurementUpdate(*z, pose_, R_, Q_, slam_phase);
            if ((p->unknown_features_.size() + p->landmarks_.size()) > 70)
            {
                //ROS_ERROR("error");
                //exit(0);
            }
        }
    }
    //ROS_INFO("data associations done");
    for(int i = 0; i != input_size; ++i)
    {
        input_data_.pop_front();
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
            //add new landmark to landmark vector and to landmark cloud for visualization
            Eigen::Vector2f lm_xy;
            lm_xy = p->addNewLandmark(u->measurement, u->colour, R_);
            geometry_msgs::Point32 lm_point;
            lm_point.x = lm_xy(0);
            lm_point.y = lm_xy(1);
            lm_point.z = 0.0;
            landmark_cloud_.points.push_back(lm_point);
            landmark_cloud_.header.stamp = ros::Time::now();
            landmark_cloud_.header.frame_id = "track";

            sensor_msgs::PointCloud2 cloud;
            sensor_msgs::convertPointCloudToPointCloud2(landmark_cloud_, cloud);
            landmark_cloud_pub_.publish(cloud);
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
    //std::cout << "distance: " << (map_feature - landmarks_.at(best_arg).mu_).norm() << std::endl;
    //std::cout << "data association value:" << best_p << std::endl;

    if ((best_p < 0.0000001) && ((map_feature - landmarks_.at(best_arg).mu_).norm() > 1.5))
    {
        DataAssociation d;
        d.measurement = measurement;
        d.landmark_id = best_arg;
        unknown_features_.push_back(d);
        //ROS_ERROR("new feature");
    }
    else
    {
        DataAssociation d;
        d.measurement = measurement;
        d.landmark_id = best_arg;
        known_features_.push_back(d);
        //ROS_ERROR("known feature");
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
    //std::cout << "uniform numbers\n" << uniform_randoms << std::endl;
    Eigen::Vector3f x_sample(cholesky_L * uniform_randoms * 0.1 + mu_);
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
    //ROS_WARN("resampling");
    //normalize weights
    normaliseWeights();

    //vector of weights
    Eigen::ArrayXf weights(num_particles_);
    int count = 0;
    std::vector<Particle>::const_iterator p;
    for (p = particles_.begin(); p != particles_.end(); ++p, ++count) {
        weights(count) = (*p).weight_;
    }
    //ROS_WARN("ciao1");
    //effective particle number and minimum number of particles
    float Neff = 1.0 / (weights.pow(2).sum());
    float Nmin = num_particles_ * resampling_ratio_;

    //vector for new set of particles
    std::vector<Particle> new_particles(num_particles_);

    //vector of cumulative weights
    Eigen::ArrayXf weights_cumulative(num_particles_);
    Eigen::ArrayXf resample_base(num_particles_);
    Eigen::ArrayXf resample_ids(num_particles_);

    //uniform distribution object
    boost::random::uniform_real_distribution<float> distribution(0.0, 1.0);

    if (Neff < Nmin)
    {
        weights_cumulative << cumulativeSum(weights);
        //std::cout << "cumul\n" << weights_cumulative << std::endl;
        //cumulative vector from 0 to [1 - (1 / NPARTICLES)] in steps of 1 / NPARTICLES
        resample_base << cumulativeSum(weights * 0.0 + 1.0 / num_particles_) - 1.0 / num_particles_;
        //std::cout << "resample base\n" << resample_base << std::endl;
        //add to every cumulative sum a number from uniform random distribution(0.0, 1.0 / NPARTICLES)
        resample_ids << resample_base + distribution(rng) / num_particles_;
        //std::cout << "resample ids\n" << resample_ids << std::endl;

        int count = 0;
        Eigen::VectorXd indexes(weights.size());
        for (int i = 0; i != num_particles_; ++i)
        {
            //see where each resample random number fits in the cumulative sum bins
            //ROS_WARN("ciao2");
            while ((count <= weights_cumulative.size() - 1) && (resample_ids(count) < weights_cumulative(i)))
            {
                //ROS_WARN("ciao3");
                //if current random number is below a cumulative sum block, add the index of the weight block
                //if random number is bigger, pass to next cumulative sum by breaking out and increasing i
                //if array size of index number is exceeded, break out of while loop and increase i until break out of for loop
                indexes(i) = count;
                ++count;
            }
        }
        //std::cout << "indexes\n" << indexes << std::endl;
        //update particles with resampled particles
        std::vector<Particle> particles_copy(particles_);
        landmark_cloud_.points.clear();
        //ROS_WARN("ciao4");
        for (int i = 0; i != indexes.size(); ++i)
        {
            //ROS_WARN("ciao5");
            particles_.at(i).mu_ = particles_copy.at(indexes(i)).mu_;
            particles_.at(i).sigma_ = particles_copy.at(indexes(i)).sigma_;
            particles_.at(i).landmarks_ = particles_copy.at(indexes(i)).landmarks_;
            //reinitialise the weights so their sum adds up to 1
            particles_.at(i).weight_ = 1.0 / num_particles_;

            std::vector<Landmark>::const_iterator lm;
            for(lm = particles_.at(i).landmarks_.begin(); lm != particles_.at(i).landmarks_.end(); ++lm)
            {
                //ROS_WARN("ciao7");
                //add new landmark to landmark vector and to landmark cloud for visualization
                /*geometry_msgs::Point32 lm_point;
                lm_point.x = lm->mu_(0);
                lm_point.y = lm->mu_(1);
                lm_point.z = 0.0;
                landmark_cloud_.points.push_back(lm_point);
                landmark_cloud_.header.stamp = ros::Time::now();
                landmark_cloud_.header.frame_id = "track";
                
                sensor_msgs::ChannelFloat32 channel;
                channel.name = "rgb";
                std::vector<float> colour(landmark_cloud_.points.size(), 25500);
                channel.values = colour;
                std::vector<sensor_msgs::ChannelFloat32> channels(landmark_cloud_.points.size(), channel);
                landmark_cloud_.channels = channels;

                sensor_msgs::PointCloud2 cloud;
                sensor_msgs::convertPointCloudToPointCloud2(landmark_cloud_, cloud);
                landmark_cloud_pub_.publish(cloud);*/
            }
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
    //ROS_WARN("final estimate");
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

    //publish pose for slam path visualization
    geometry_msgs::PoseStamped current_pose;
    current_pose.pose.position.x = xEst_(0);
    current_pose.pose.position.y = xEst_(1);
    current_pose.pose.position.z = 0.0;
    geometry_msgs::Quaternion orientation = tf::createQuaternionMsgFromYaw(xEst_(2));
    current_pose.pose.orientation = orientation;
    current_pose.header.frame_id = "track";
    current_pose.header.stamp = ros::Time::now();
    slam_path_.header = current_pose.header;
    slam_path_.poses.push_back(current_pose);
    slam_path_pub_.publish(slam_path_);

    //publish odometry message for slam estimate
    nav_msgs::Odometry est;
    est.header.stamp = ros::Time::now();
    est.header.frame_id = "track";
    est.child_frame_id = "base_footprint";

    est.pose.pose.position.x = xEst_(0);
    est.pose.pose.position.y = xEst_(1);
    est.pose.pose.position.z = 0.0;
    est.pose.pose.orientation = orientation;
    slam_estimate_pub_.publish(est);

    return xEst_;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "slam_server");
    ros::NodeHandle nh;
    StateEstimation action_server(&nh, 10, "fastslam");
    ros::spin();

    return 0;
}
