#ifndef FAST_SLAM2_ROS_HEADER
#define FAST_SLAM2_ROS_HEADER

#include <Eigen/Dense>
#include <nav_msgs/Odometry.h>
#include "particle.hpp"
#include "slam_utils.hpp"
#include <perception_pkg/Cone.h>
#include <vector>
#include <deque>
#include <eufs_msgs/WheelSpeedsStamped.h>
#include <nav_msgs/Path.h>
#include <Eigen/Core>
#include "boost/random.hpp"
#include <actionlib/server/simple_action_server.h>
#include <motion_estimation_mapping_pkg/FastSlamAction.h>
#include <sensor_msgs/PointCloud.h>
#include <visualization_msgs/MarkerArray.h>
#include "ekf_localisation.hpp"
#include <sensor_msgs/Imu.h>
#include <geodetic_to_enu_conversion_pkg/Gps.h>
#include <nav_msgs/Path.h>
#include <sensor_msgs/PointCloud.h>

class StateEstimation
{

public:
    /**
     * @brief Constructor of the StateEstimation class: the action server is created, the action server takes 
     * arguments of a node handle, name of the action and an executeCB; additionally FastSLAM2.0 parameters are 
     * initialised such as a private node handle for the ROS parameter server. The initialise() function is called 
     * to perform most of the class parameter initilialisations and based on the num_particles, a vector of empty
     * particles is also initialised.
     * @param nh NodeHandle pointer 
     * @param num_particles Number of particles to be used in the FastSLAM2.0 algorithm
     * @param name  Name of the action server 
     * @return StateEstimation class object with initialised parameters and initialised action server
     */
    StateEstimation(ros::NodeHandle* nh, int num_particles, const std::string& name): num_particles_(num_particles),
        ground_truth_(Eigen::Vector3f::Zero()), private_nh_("~"), nh_(*nh), action_name_(name),
        as_(*nh, name, boost::bind(&StateEstimation::executeCB, this, _1), false)
        {
            initialise();
            as_.start();

            for (int i = 0; i != num_particles; ++i)
            {
                Particle particle;
                particles_.push_back(particle);
            }
        };

    /**
     * @brief Callback function executes the goal it is passed 
     * @param goal FastSlamGoal pointer to the goal message sent to the action servee
     */
    void executeCB(const motion_estimation_mapping_pkg::FastSlamGoal::ConstPtr& goal);

    /**
     * @brief Initialises the StateEstimation object with parameters, publishers and subscribers needed for FastSLAM
     */
    void initialise();
    
    /**
     * @brief Sampling step: applying the motion model to every particle
     * @param ekf EKF class object reference used to apply the Extended Kalman Filter step for the motion model
     */
    void prediction(EKF& ekf);

    /**
     * @brief Correction step: applying the measurement model to every particle
     * @param slam_phase The phase (mapping or localization) the car is currently executing 
     */
    void correction(SLAM_PHASE slam_phase);

    /**
     * @brief Resamples the particles based on the updated weights from the correction step
     */
    void resampling();

    /**
     * @brief Updates the SLAM estimate with the re-sampled particles and updated weights 
     * @return A 3D Eigen Vector representing the final SLAM estimate for the current iteration
     */
    Eigen::Vector3f calculateFinalEstimate();
    
private:
    /**
     * @brief Initialises subscribers needed for the FastSLAM2.0 algorithm 
     */
    void initialiseSubscribers();

    /**
     * @brief Initialises publishers needed for the FastSLAM2.0 algorithm 
     */
    void initialisePublishers();

    /**
     * @brief Normalizes the particle weights
     */
    void normaliseWeights();

    /**
     * @brief Callback for receiving odometry data
     * @param msg An Odometry message
     */
    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);

    /**
     * @brief Callback for receiving ground truth cone location data
     * @param msg A MarkerArray message
     */
    void groundTruthConeCallback(const visualization_msgs::MarkerArray::ConstPtr& msg);

    /**
     * @brief Callback for receiving detected Cone data
     * @param msg A Cone message
     */
    void coneCallback(const perception_pkg::Cone::ConstPtr& msg);

    /**
     * @brief Callback for receiving WheelSpeeds data
     * @param msg A WheelSpeeds message
     */
    void wheelSpeedCallback(const eufs_msgs::WheelSpeedsStamped::ConstPtr& msg);

    /**
     * @brief Callback for receiving converted NavSatFix to Gps data 
     * @param msg A Gps message
     */
    void gpsCallback(const geodetic_to_enu_conversion_pkg::Gps::ConstPtr& msg);

    /**
     * @brief Callback for receiving Imu data 
     * @param msg An Imu message
     */
    void imuCallback(const sensor_msgs::Imu::ConstPtr& msg);

    boost::random::mt19937 rng;

    //action server variables 
    actionlib::SimpleActionServer<motion_estimation_mapping_pkg::FastSlamAction> as_;
    std::string action_name_;
    motion_estimation_mapping_pkg::FastSlamFeedback feedback_;
    motion_estimation_mapping_pkg::FastSlamResult result_;   

    //ROS node handles 
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;

    //ROS subscribers 
    ros::Subscriber odom_sub_;
    ros::Subscriber wheel_speeds_sub_;
    ros::Subscriber cone_sub_;
    ros::Subscriber gps_sub_;
    ros::Subscriber imu_sub_;
    ros::Subscriber ground_truth_cone_sub_;

    //ROS publishers
    ros::Publisher odom_pub_;
    ros::Publisher odom_path_pub_;
    ros::Publisher pred_path_pub_;
    ros::Publisher cone_array_pub_;
    ros::Publisher slam_path_pub_;
    ros::Publisher slam_estimate_pub_;
    ros::Publisher landmark_cloud_pub_;

    //vehicle properties and vehicle dynamics parameters 
    float wheel_base_; 
    float wheel_diameter_;
    float max_speed_;
    float max_steering_;

    //Time variables 
    ros::Time last_time_, current_time_;

    //GPS x and y locations, and IMU yaw euler orientation
    float gps_x_, gps_y_;
    float imu_yaw_;

    //FastSLAM2.0 configuration parameters  
    const int num_particles_;
    float resampling_ratio_;

    //control and measurement noise covariance matrix coefficients
    float sigmaV, sigmaG;
    float sigmaR, sigmaB;

    //estimate variables 
    Eigen::Vector3f xEst_;    
    
    //FastSLAM algorithm noise 
    Eigen::Matrix2f R_; //linearized vehicle measurement noise 
    Eigen::Matrix2f Q_; //linearized vehicle control noise 

    //FastSLAM2.0 vector of particles
    std::vector<Particle> particles_;

    //observations
    std::deque<perception_pkg::Cone> input_data_;

    //helper booleans 
    bool lap_closure_detected_;
    bool start_;

    //ground truth odometry estimate
    Eigen::Vector3f ground_truth_;

    //robot motion input commands
    RobotCommand u_;

    //path visaualization variables 
    nav_msgs::Path odom_path_;
    nav_msgs::Path pred_path_;
    nav_msgs::Path slam_path_;

    //ground truth cone visualization variables
    int cone_counter_;
    visualization_msgs::MarkerArray cone_array_;

    //landmark visualization variable
    sensor_msgs::PointCloud landmark_cloud_;
};

#endif