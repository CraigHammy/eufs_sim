#ifndef FAST_SLAM_ROS_HEADER
#define FAST_SLAM_ROS_HEADER

#include <Eigen/Dense>
#include <nav_msgs/Odometry.h>
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

class StateEstimation
{

public:
    StateEstimation(ros::NodeHandle* nh, int num_particles, int update_frequency): num_particles_(num_particles), state_size_(3),
        lm_size_(2), num_particles_resampling_(num_particles / 1.5), update_frequency_(update_frequency), pose_(Eigen::Vector3f::Zero()), nh_(*nh)
        {
            for (int i = 0; i != num_particles; ++i)
            {
                Particle particle;
                particles_.push_back(particle);
            }
        };

    ros::Time last_time_, current_time_;

    /**
     * @brief Initialise a StateEstimation object and the Particles objects
     */
    void initialise();

    /**
     * @brief Runs the FastSLAM2.0 algorithm
     * @param observations List of Cone messages 
     */
    void FastSLAM2(const std::vector<perception_pkg::Cone>& observations);
    
    /**
     * @brief Sampling step: applying the motion model to every particle
     */
    void prediction();

    /**
     * @brief Correction step: applying the measurement model to every particle
     * @param observations A list of Cone messages 
     * @param slam_phase The phase (mapping or localization) the car is currently executing 
     */
    void correction(const std::vector<perception_pkg::Cone>& observations, SLAM_PHASE slam_phase);

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

    void initialiseSubscribers();

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
     * @brief Callback for receiving command data
     * @param msg An AckermannDriveStamped message
     */
    void commandCallback(const ackermann_msgs::AckermannDriveStamped::ConstPtr& msg);

    /**
     * @brief Callback for receiving joint state data
     * @param msg A JointState message
     */
    void jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg);

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

    boost::random::mt19937 rng;

    //ROS variables 
    ros::NodeHandle nh_;
    ros::Subscriber odom_sub_;
    ros::Subscriber command_sub_;
    ros::Subscriber joint_states_sub_;
    ros::Subscriber wheel_speeds_sub_;

    //vehicle wheelbase
    float wheel_base_; 
    float wheel_diameter_;
    float max_speed_;
    float max_steering_;

    //FastSLAM configuration parameters  
    const int num_particles_;
    const int state_size_;
    const int lm_size_;
    const int num_particles_resampling_;
    float resampling_ratio_;

    //estimate variables 
    Eigen::Vector3f xEst_;
    const int update_frequency_;

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

    //visaualization paths 
    ros::Publisher odom_path_pub_;
    nav_msgs::Path odom_path_;
    ros::Publisher pred_path_pub_;
    nav_msgs::Path pred_path_;

    
    //FastSLAM algorithm noise 
    Eigen::Matrix2f R_; //linearized vehicle measurement noise 
    Eigen::Matrix2f Q_; //linearized vehicle control noise 
};

#endif