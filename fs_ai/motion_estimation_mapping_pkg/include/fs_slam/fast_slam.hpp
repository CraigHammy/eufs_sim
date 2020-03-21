#ifndef FAST_SLAM2_ROS_HEADER
#define FAST_SLAM2_ROS_HEADER

#include <Eigen/Dense>
#include <nav_msgs/Odometry.h>
#include "particle.hpp"
#include "slam_utils.hpp"
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <sensor_msgs/JointState.h>
#include <perception_pkg/Cone.h>
#include <vector>
#include <eufs_msgs/WheelSpeedsStamped.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <Eigen/Core>
#include "boost/random.hpp"
#include <actionlib/server/simple_action_server.h>
#include <motion_estimation_mapping_pkg/FastSlamAction.h>
#include <sensor_msgs/PointCloud.h>
#include <visualization_msgs/MarkerArray.h>
#include "ekf_localisation.hpp"
#include <sensor_msgs/Imu.h>
#include <geodetic_to_enu_conversion_pkg/Gps.h>

class StateEstimation
{

public:
    StateEstimation(ros::NodeHandle* nh, int num_particles, const std::string& name): num_particles_(num_particles),
        num_particles_resampling_(num_particles / 1.5), pose_(Eigen::Vector3f::Zero()), private_nh_("~"), nh_(*nh), action_name_(name),
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

    void executeCB(const motion_estimation_mapping_pkg::FastSlamGoal::ConstPtr& goal)
    {
        EKF ekf(&private_nh_);

        while(!lap_closure_detected_)
        {
            if (start_)
            {
                prediction(ekf);
                //correction(MAP_BUILDING);
                calculateFinalEstimate();
                //resampling();
            }
        }
    }

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
     * @brief
     * @param
     * @return
     */
    void addNewLandmark();

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

    //ROS variables 
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
    ros::Subscriber odom_sub_;
    ros::Subscriber command_sub_;
    ros::Subscriber joint_states_sub_;
    ros::Subscriber wheel_speeds_sub_;
    ros::Subscriber cone_sub_;
    ros::Subscriber gps_sub_;
    ros::Subscriber imu_sub_;

    //vehicle wheelbase
    float wheel_base_; 
    float wheel_diameter_;
    float max_speed_;
    float max_steering_;

    ros::Time last_time_, current_time_;

    //FastSLAM configuration parameters  
    const int num_particles_;
    const int num_particles_resampling_;
    float resampling_ratio_;

    //estimate variables 
    Eigen::Vector3f xEst_;

    //Initialise vector of particles
    std::vector<Particle> particles_;

    //SLAM phase switch 
    bool lap_closure_detected_;

    //position and velocity odometric estimates
    Eigen::Vector3f pose_;

    //robot motion commands
    RobotCommand u_;

    //robot joint states variables 
    sensor_msgs::JointState joint_state_;
    bool read_state_;
    int frw_pos_, flw_pos_, brw_vel_, blw_vel_;
    bool start_;

    ros::Publisher odom_pub_;

    //visaualization paths 
    ros::Publisher odom_path_pub_;
    nav_msgs::Path odom_path_;
    ros::Publisher pred_path_pub_;
    nav_msgs::Path pred_path_;
    ros::Publisher slam_path_pub_;
    nav_msgs::Path slam_path_;

    ros::Publisher cone_array_pub_;
    int cone_counter_;
    visualization_msgs::MarkerArray cone_array_;
    ros::Subscriber ground_truth_cone_sub_;

    //slam estimate pose
    ros::Publisher slam_estimate_pub_;
    
    //FastSLAM algorithm noise 
    Eigen::Matrix2f R_; //linearized vehicle measurement noise 
    Eigen::Matrix2f Q_; //linearized vehicle control noise 

    //observations
    std::deque<perception_pkg::Cone> input_data_;

    //landmark visualization
    sensor_msgs::PointCloud landmark_cloud_;
    ros::Publisher landmark_cloud_pub_;

    //GPS x and y locations, and IMU yaw euler orientation
    float gps_x_, gps_y_;
    float imu_yaw_;

    //control and measurement noise covariance matrix coefficients
    float sigmaV, sigmaG;
    float sigmaR, sigmaB;
    
};

#endif