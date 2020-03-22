#include "slam_utils.hpp"
#include "particle.hpp"
#include "fast_slam.hpp"
#include <Eigen/Dense>
#include <ros/ros.h>
#include <cmath>
#include <vector>
#include <deque>
#include <algorithm>
#include <string>
#include <iostream>
#include <stdint.h>
#include <ctime>
#include <geometry_msgs/Point.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <std_msgs/Header.h>
#include <eufs_msgs/WheelSpeedsStamped.h>
#include <Eigen/Cholesky>
#include <Eigen/Core>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <motion_estimation_mapping_pkg/FastSlamAction.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <sensor_msgs/ChannelFloat32.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include "ekf_localisation.hpp"

std::time_t now = std::time(0);
boost::random::mt19937 rng(static_cast<uint32_t>(now));

/**
 * @brief Initialises the StateEstimation object with parameters, publishers and subscribers needed for FastSLAM
 */
void StateEstimation::initialise()
{
    //initialise helper variables 
    control_start_ = false;
    gps_start_ = false;
    imu_start_ = false;
    cone_counter_ = 0;
    lap_closure_detected_ = false;

    //initialise publishers and publishers 
    initialiseSubscribers();
    initialisePublishers();

    //instantiate time variables needed to work out time differences between FastSLAM2.0 iterations
    current_time_ = ros::Time::now();
    last_time_ = current_time_;

    //load parameters from ROS Param Server
    private_nh_.getParam("max_speed", max_speed_);
    private_nh_.getParam("max_steering", max_steering_);
    private_nh_.getParam("wheel_base", wheel_base_);
    private_nh_.getParam("wheel_diameter", wheel_diameter_);
    private_nh_.getParam("slam_control_noise_velocity", sigmaV);
    private_nh_.getParam("slam_control_noise_steering_angle", sigmaG);
    private_nh_.getParam("slam_measurement_noise_euclidean_distance", sigmaR);
    private_nh_.getParam("slam_measurement_noise_angle_difference", sigmaB);
    private_nh_.getParam("resampling_ratio_particles", resampling_ratio_);

    //initialise control noise covariance matrix
    Q_ << powf(sigmaV, 2), 0, 0, powf(sigmaG * M_PI / 180, 2);

    //initialise measurement noise covariance matrix
    R_ << powf(sigmaR, 2), 0, 0, powf(sigmaB * M_PI / 180, 2);

    ROS_WARN("FastSLAM2.0 node has been initialised");
}

/**
 * @brief Initialises publishers needed for the FastSLAM2.0 algorithm 
 */
void StateEstimation::initialisePublishers()
{
    odom_path_pub_ = nh_.advertise<nav_msgs::Path>("odom_path", 1);
    pred_path_pub_ = nh_.advertise<nav_msgs::Path>("prediction_path", 1);
    slam_path_pub_ = nh_.advertise<nav_msgs::Path>("slam_path", 1);
    landmark_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/landmark_cloud", 1);
    slam_estimate_pub_ = nh_.advertise<nav_msgs::Odometry>("/slam_estimate", 1);
    cone_array_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/cone_markers", 1);
    //odom_pub_ = nh_.advertise<visualization_msgs::Marker>("/odom_marker", 1);
}

/**
 * @brief Initialises subscribers needed for the FastSLAM2.0 algorithm 
 */
void StateEstimation::initialiseSubscribers()
{
    odom_sub_ = nh_.subscribe<nav_msgs::Odometry>("/ground_truth/odom", 1, boost::bind(&StateEstimation::odomCallback, this, _1));
    wheel_speeds_sub_ = nh_.subscribe<eufs_msgs::WheelSpeedsStamped>("/ros_can/wheel_speeds", 1, boost::bind(&StateEstimation::wheelSpeedCallback, this, _1));
    cone_sub_ = nh_.subscribe<perception_pkg::Cone>("/cones", 1, boost::bind(&StateEstimation::coneCallback, this, _1));
    ground_truth_cone_sub_ = nh_.subscribe<visualization_msgs::MarkerArray>("/ground_truth/cones/viz", 1, boost::bind(&StateEstimation::groundTruthConeCallback, this, _1));
    gps_sub_ = nh_.subscribe<geodetic_to_enu_conversion_pkg::Gps>("/odometry/gps", 1, boost::bind(&StateEstimation::gpsCallback, this, _1));
    imu_sub_ = nh_.subscribe<sensor_msgs::Imu>("/imu", 1, boost::bind(&StateEstimation::imuCallback, this, _1));
}

/**
 * @brief Callback function executes the goal it is passed 
 * @param goal FastSlamGoal pointer to the goal message sent to the action servee
 */
void StateEstimation::executeCB(const motion_estimation_mapping_pkg::FastSlamGoal::ConstPtr& goal)
{
    while(!lap_closure_detected_)
    {
        //ros::Time start = ros::Time::now();
        if (control_start_ && gps_start_ && imu_start_)
        {
            prediction();
            correction(MAP_BUILDING);
            calculateFinalEstimate();
            resampling();
        }
        //ros::Time finish = ros::Time::now();
        //std::cout << "frequency: " << 1/((finish - start).toSec()) << std::endl;
    }

    while (ros::ok)
    {
        std::vector<Particle>::iterator p;
        for (p = particles_.begin(); p != particles_.end(); ++p)
        {
            std::vector<Landmark>::const_iterator lm;
            for(lm = p->landmarks_.begin(); lm != p->landmarks_.end(); ++lm)
            {
                //add new landmark to landmark vector and to landmark cloud for visualization
                geometry_msgs::Point32 point;
                point.x = lm->mu_(0);
                point.y = lm->mu_(1);
                point.z = 0.0;
                landmark_cloud_.points.push_back(point);
                landmark_cloud_.header.stamp = ros::Time::now();
                landmark_cloud_.header.frame_id = "track";
                
                sensor_msgs::ChannelFloat32 channel;
                channel.name = "rgb";
                std::vector<float> colour(landmark_cloud_.points.size(), 255255255);
                channel.values = colour;
                std::vector<sensor_msgs::ChannelFloat32> channels(landmark_cloud_.points.size(), channel);
                landmark_cloud_.channels = channels;

                //publish landmark PointCloud2 message 
                sensor_msgs::PointCloud2 cloud;
                sensor_msgs::convertPointCloudToPointCloud2(landmark_cloud_, cloud);
                landmark_cloud_pub_.publish(cloud);
            }
        }
    }
    as_.setSucceeded();
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
        ROS_INFO("division by zero occcurred");
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
    control_start_ = true;
    float rpm_rb = msg->rb_speed; 
    float rpm_lb = msg->lb_speed;
    float rpm_rf = msg->rf_speed;
    float rpm_lf = msg->lf_speed;

    //transform from RPM to m/s
    float right_back = 2 * M_PI * (wheel_diameter_/2) * rpm_rb / 60;
    float left_back = 2 * M_PI * (wheel_diameter_/2) * rpm_lb / 60;

    //update RobotCommand variable 
    u_.speed = (right_back + left_back) / 2.0;
    u_.steering_angle = msg->steering;

    if ((u_.speed < 0.01) && (particles_.at(0).landmarks_.size() > 30)) {
        ROS_WARN("lap closure detected");
        lap_closure_detected_ = true;
    }
}

/**
 * @brief Callback for receiving odometry data
 * @param msg An Odometry message
 */
void StateEstimation::odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    //odometry posiion x and y
    ground_truth_(0) = msg->pose.pose.position.x;
    ground_truth_(1) = msg->pose.pose.position.y;

    //transform yaw from quaternion to euler 
    tf::Quaternion q(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, 
        msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
    double roll, pitch, yaw;
    tf::Matrix3x3 m(q);
    m.getRPY(roll, pitch, yaw);
    ground_truth_(2) = yaw;

    //check if visualization is requested by user
    if (vis_)
    {
        //store retrieved pose into Path message and publish it 
        geometry_msgs::PoseStamped current_pose;
        current_pose.pose.position.x = ground_truth_(0);
        current_pose.pose.position.y = ground_truth_(1);
        current_pose.pose.position.z = 0;
        tf::Quaternion orientation;
        orientation.setRPY(0, 0, ground_truth_(2));

        current_pose.header.frame_id = "track";
        current_pose.header.stamp = ros::Time::now();
        
        odom_path_.header = current_pose.header;
        odom_path_.poses.push_back(current_pose);
        odom_path_pub_.publish(odom_path_);
    }
}

/**
 * @brief Callback for receiving ground truth cone location data
 * @param msg A MarkerArray message
 */
void StateEstimation::groundTruthConeCallback(const visualization_msgs::MarkerArray::ConstPtr& msg)
{
    //store MarkerArray message 
    visualization_msgs::MarkerArray array;
    array = *msg;

    //if using a rosbag change the time of the Header, else remove this loop
    std::vector<visualization_msgs::Marker>::iterator iter;
    for (iter = array.markers.begin(); iter != array.markers.end(); ++iter)
        iter->header.stamp = ros::Time::now();

    //publish the MarkerArray
    cone_array_pub_.publish(array);
}

/**
 * @brief Callback for receiving detected Cone data
 * @param msg A Cone message
 */
void StateEstimation::coneCallback(const perception_pkg::Cone::ConstPtr& msg)
{
    //add cone detection to input data array used in the correction step of the FastSLAM2.0 algorithm
    input_data_.push_back(*msg);

    //if detected cone is orange, lap closure detected
    std::string colour = msg->colour;
    std::cout << colour << std::endl;
    if ((colour == "orange") && (particles_.at(0).landmarks_.size() > 20)) {
        ROS_WARN("lap closure detected");
        std::cout << msg->colour << std::endl;
        lap_closure_detected_ = true;
    }
    
   
    //publish the cone detections as markers
    /*visualization_msgs::Marker marker;
    marker.header.frame_id = "/velodyne";
    marker.header.stamp = ros::Time::now();
    marker.id = cone_counter_;
    marker.type = visualization_msgs::Marker::CUBE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = msg->location.x;
    marker.pose.position.y = msg->location.y;
    marker.pose.position.z = msg->location.z;
    marker.scale.x = 0.15;
    marker.scale.y = 0.15;
    marker.scale.z = 0.15;
    marker.color.r = 1.0f;
    marker.color.g = 0.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0;
    cone_array_.markers.push_back(marker);
    cone_array_pub_.publish(cone_array_);
    ++cone_counter_;*/
}

/**
 * @brief Callback for receiving converted NavSatFix to Gps data 
 * @param msg A Gps message
 */
void StateEstimation::gpsCallback(const geodetic_to_enu_conversion_pkg::Gps::ConstPtr& msg)
{
    //GPS x and y positions in ENU (East-North-Up) format
    gps_start_ = true;
    gps_x_ = float(msg->x);
    gps_y_ = float(msg->y);
}

/**
 * @brief Callback for receiving Imu data 
 * @param msg An Imu message
 */
void StateEstimation::imuCallback(const sensor_msgs::Imu::ConstPtr& msg)
{
    //convert yaw angle from quaternion to euler 
    imu_start_ = true;
    tf::Quaternion q(msg->orientation.x, msg->orientation.y,
                    msg->orientation.z, msg->orientation.w);
    tf::Matrix3x3 m(q);
    double pitch, roll, yaw;
    m.getRPY(roll, pitch, yaw);
    imu_yaw_ = yaw;
}


/**
 * @brief Sampling step: applying the motion model to every particle
 * @param ekf EKF class object reference used to apply the Extended Kalman Filter step for the motion model
 */
void StateEstimation::prediction()
{
    //ROS_WARN("FastSLAM2.0 PREDICTION step underway...");

    //store values from the WheelSpeeds callback  
   float steering_angle = u_.steering_angle;
   float speed = u_.speed;

    //cap the linear velocity and steering angle at a minimum and a maximum
   if (speed > max_speed_) speed = max_speed_;
   if (speed < -max_speed_) speed = -max_speed_;
   if (steering_angle > max_steering_) steering_angle = max_steering_;
   if (steering_angle < -max_steering_) steering_angle = -max_steering_;

    //time step update
    current_time_ = ros::Time::now();
    float dt = current_time_.toSec() - last_time_.toSec();
    last_time_ = current_time_;

    //create observation vector from GPS and IMU readings 
    Eigen::Vector3f z;
    z << gps_x_, gps_y_, imu_yaw_;

    //create control input vector needed for Extended Kalman Filter
    Eigen::Vector2f u(speed, steering_angle);

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

        //motion update using the Extended Kalman Filter step (prediction and update/correction)
        Estimates e = p->ekf_.ekf_estimation_step(u, dt, wheel_base_, z);
        p->mu_ = e.xEst;

        //covariance update
        p->sigma_ = Gx * p->sigma_ * Gx.transpose() + Gu * Q_ * Gu.transpose();

        //EKF motion model state vector (before GPS/IMU measurement update)
        p->mu_pred_ = e.xPred;
    }

    //check if visualization is requested by user
    if (vis_)
    {
        //add the EKF motion model prediction pose to a Path message and publish it (to see difference with ground truth and SLAM estimate)
        float x = 0.0;
        float y = 0.0;
        float yaw = 0.0;
        for (p = particles_.begin(); p != particles_.end(); ++p)
        {
            x += p->mu_pred_(0);
            y += p->mu_pred_(1);
            yaw += p->mu_pred_(2);
        }
        geometry_msgs::PoseStamped current_pose;
        current_pose.pose.position.x = x / num_particles_;
        current_pose.pose.position.y = y / num_particles_;
        current_pose.pose.position.z = 0.0;
        geometry_msgs::Quaternion orientation = tf::createQuaternionMsgFromYaw(yaw / num_particles_);
        current_pose.pose.orientation = orientation;

        current_pose.header.frame_id = "track";
        current_pose.header.stamp = ros::Time::now();
        
        pred_path_.header = current_pose.header;
        pred_path_.poses.push_back(current_pose);
        pred_path_pub_.publish(pred_path_);
    }
}

/**
 * @brief Correction step: applying the measurement model to every particle
 * @param slam_phase The phase (mapping or localization) the car is currently executing
 */
void StateEstimation::correction(SLAM_PHASE slam_phase)
{
    //ROS_WARN("FastSLAM2.0 CORRECTION step underway...");

    std::vector<Particle>::iterator p;
    std::deque<perception_pkg::Cone>::const_iterator z;
    std::deque<perception_pkg::Cone> observations(input_data_.begin(), input_data_.end());
    int input_size = observations.size();

    for(z = observations.begin(); z != observations.end(); ++z)
    {
        for (p = particles_.begin(); p != particles_.end(); ++p)
        {
            //divide unknown from known features and perform data association on known features
            p->measurementUpdate(*z, R_, slam_phase);
        }
    }
    
    //remove cone detectiond data from the input data array to make space for new cone data 
    for(int i = 0; i != input_size; ++i)
    {
        input_data_.pop_front();
    }

    //iterating through all particles
    int particle_count = 1;
    for (p = particles_.begin(); p != particles_.end(); ++p, ++particle_count)
    {
        if (p->known_features_.size() != 0)
        {
            //update proposal distribution and then sample from proposal distribution 
            p->proposalSampling(p->known_features_, R_);

            //iterating through known features 
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
            //remove all data from known features vector 
            p->known_features_.clear();
        }

        //iterating through unknown features
        std::vector<DataAssociation>::const_iterator u;
        for (u = p->unknown_features_.begin(); u != p->unknown_features_.end(); ++u)
        {   
            //add a new landmark
            Eigen::Vector2f landmark;
            landmark = p->addNewLandmark(u->measurement, u->colour, R_);
            /*
            //add to mappings to keep track of the pointcloud population
            int landmark_number = p->landmarks_.size() + 1;
            std::stringstream result;
            std::vector<int> encoding;
            encoding.push_back(0);
            encoding.push_back(particle_count);
            encoding.push_back(0);
            encoding.push_back(landmark_number);
            std::copy(encoding.begin(), encoding.end(), std::ostream_iterator<int>(result, ""));
            landmark_vis_mappings_.insert(std::pair<std::string, int>(result.str().c_str(), landmark_cloud_.points.size()));
            //ROS_INFO("key: %s, value: %d", result.str().c_str(), int(landmark_cloud_.points.size()));

            //add new landmark to landmark cloud for visualization
            geometry_msgs::Point32 point;
            point.x = landmark(0);
            point.y = landmark(1);
            point.z = 0.0;
            landmark_cloud_.points.push_back(point);
            landmark_cloud_.header.stamp = ros::Time::now();
            landmark_cloud_.header.frame_id = "track";*/
        }
        //remove all data from unknown features vector 
        p->unknown_features_.clear();
    }
    //publish landmark PointCloud2 message
    //sensor_msgs::PointCloud2 cloud;
    //sensor_msgs::convertPointCloudToPointCloud2(landmark_cloud_, cloud);
    //landmark_cloud_pub_.publish(cloud);
}


/**
 * @brief Resamples the particles based on the updated weights from the correction step
 */
void StateEstimation::resampling()
{
    //ROS_WARN("FastSLAM2.0 RESAMPLING step underway...");

    //normalize weights
    normaliseWeights();

    //vector of weights
    Eigen::ArrayXf weights(num_particles_);
    int count = 0;
    std::vector<Particle>::const_iterator p;
    for (p = particles_.begin(); p != particles_.end(); ++p, ++count) {
        weights(count) = p->weight_;
    }

    //effective particle number and minimum number of particles
    float Neff = 1.0 / (weights.pow(2).sum());
    float Nmin = num_particles_ * resampling_ratio_;

    //vector for new set of particles
    std::vector<Particle> new_particles;
    for (int i = 0; i != num_particles_; ++i)
    {
        EKF ekf(&private_nh_);
        Particle p(&ekf);
        new_particles.push_back(p);

    }

    //vector of cumulative weights
    Eigen::ArrayXf weights_cumulative(num_particles_);
    Eigen::ArrayXf resample_base(num_particles_);
    Eigen::ArrayXf resample_ids(num_particles_);

    //uniform distribution object
    boost::random::uniform_real_distribution<float> distribution(0.0, 1.0);

    if (Neff < Nmin)
    {
        weights_cumulative << cumulativeSum(weights);
        //cumulative vector from 0 to [1 - (1 / NPARTICLES)] in steps of 1 / NPARTICLES
        resample_base << cumulativeSum(weights * 0.0 + 1.0 / num_particles_) - 1.0 / num_particles_;
        //add to every cumulative sum a number from uniform random distribution(0.0, 1.0 / NPARTICLES)
        resample_ids << resample_base + distribution(rng) / num_particles_;

        int count = 0;
        Eigen::VectorXd indexes(num_particles_);
        for (int i = 0; i != num_particles_; ++i)
        {
            //see where each resample random number fits in the cumulative sum bins
            while ((count <= weights_cumulative.size() - 1) && (resample_ids(count) < weights_cumulative(i)))
            {
                //if current random number is below a cumulative sum block, add the index of the weight block
                //if random number is bigger, pass to next cumulative sum by breaking out and increasing i
                //if array size of index number is exceeded, break out of while loop and increase i until break out of for loop
                indexes(count) = i;
                ++count;
            }
        }

        //update particles with resampled particles
        std::vector<Particle> particles_copy(particles_);
        landmark_cloud_.points.clear();

        for (int i = 0; i != indexes.size(); ++i)
        {
            particles_.at(i).mu_ = particles_copy.at(indexes(i)).mu_;
            particles_.at(i).sigma_ = particles_copy.at(indexes(i)).sigma_;
            particles_.at(i).landmarks_ = particles_copy.at(indexes(i)).landmarks_;

            //reinitialise the weights so their sum adds up to 1
            particles_.at(i).weight_ = 1.0 / num_particles_;
/*
            std::vector<Landmark>::const_iterator lm;
            for(lm = particles_.at(i).landmarks_.begin(); lm != particles_.at(i).landmarks_.end(); ++lm)
            {
                
                //add new landmark to landmark vector and to landmark cloud for visualization
                geometry_msgs::Point32 point;
                point.x = lm->mu_(0);
                point.y = lm->mu_(1);
                point.z = 0.0;
                landmark_cloud_.points.push_back(point);
                landmark_cloud_.header.stamp = ros::Time::now();
                landmark_cloud_.header.frame_id = "track";
                
                sensor_msgs::ChannelFloat32 channel;
                channel.name = "rgb";
                std::vector<float> colour(landmark_cloud_.points.size(), 255255255);
                channel.values = colour;
                std::vector<sensor_msgs::ChannelFloat32> channels(landmark_cloud_.points.size(), channel);
                landmark_cloud_.channels = channels;

                //publish landmark PointCloud2 message 
                sensor_msgs::PointCloud2 cloud;
                sensor_msgs::convertPointCloudToPointCloud2(landmark_cloud_, cloud);
                ROS_INFO("hello");
                landmark_cloud_pub_.publish(cloud);
                
            }
            */
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
    //ROS_INFO("FastSLAM2.0 Final Estimate");
    xEst_ = Eigen::Vector3f::Zero();
    normaliseWeights();

    //since all weights add up to 1, add from 0 the different weighted components of the state of each particle
    std::vector<Particle>::iterator p;
    
    int j = 1;
    for (p = particles_.begin(); p != particles_.end(); ++p, ++j)
    {
        if (p->weight_ != p->weight_)
        {
            ROS_ERROR("NaN weight value");
            exit(0);
        }
        xEst_(0) += p->weight_ * p->mu_(0);
        xEst_(1) += p->weight_ * p->mu_(1);
        xEst_(2) = angleWrap(xEst_(2) + p->weight_ * p->mu_(2));
    }

    //check if visualization is requested by user
    if (vis_)
    {
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
        /*nav_msgs::Odometry est;
        est.header.stamp = ros::Time::now();
        est.header.frame_id = "track";
        est.child_frame_id = "base_footprint";

        est.pose.pose.position.x = xEst_(0);
        est.pose.pose.position.y = xEst_(1);
        est.pose.pose.position.z = 0.0;
        est.pose.pose.orientation = orientation;
        slam_estimate_pub_.publish(est);*/
    }

    return xEst_;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "slam_server");
    ros::NodeHandle nh;
    
    //initialise FastSLAM2.0 action server and keep it open
    StateEstimation action_server(&nh, 1, "fastslam");
    ros::spin();

    return 0;
}
