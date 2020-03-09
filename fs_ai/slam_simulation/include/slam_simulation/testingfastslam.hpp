#ifndef FAST_SLAM_ROS_HEADER2
#define FAST_SLAM_ROS_HEADER2

#include <Eigen/Dense>
#include "test_slam_utils.hpp"
#include "testingparticle.hpp"
#include <vector>
#include <Eigen/Core>
#include "ros/ros.h"
#include "boost/random.hpp"
#include <nav_msgs/Path.h>
#include <visualization_msgs/MarkerArray.h>

class TestStateEstimation
{

public:
    TestStateEstimation(int num_particles, int num_landmarks, int update_frequency): num_particles_(num_particles), state_size_(3),
        num_landmarks_(num_landmarks), lm_size_(2), num_particles_resampling_(num_particles / 1.5),
        update_frequency_(update_frequency), pose_(Eigen::Vector3f::Zero()) 
        {
            for (int i = 0; i != num_particles; ++i)
            {
                TestParticle particle;
                particles_.push_back(particle);
            }
        };

    /**
     * @brief Initialise a StateEstimation object and the Particles objects
     */
    void initialise();

    /**
     * @brief Runs the FastSLAM2.0 algorithm
     * @param observations List of Cone messages
     */
    void FastSLAM2(const std::vector<ConeLocation>& observations);

    /**
     * @brief Sampling step: applying the motion model to every particle
     */
    void prediction(Eigen::Vector2f u);

    /**
     * @brief Correction step: applying the measurement model to every particle
     * @param observations A list of Cone messages
     * @param slam_phase The phase (mapping or localization) the car is currently executing
     */
    void correction(const std::vector<ConeLocation>& observations, TEST_SLAM_PHASE slam_phase);

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
     * @brief Normalizes the particle weights
     */
    void normaliseWeights();

    //static std::default_random_engine rng;
    boost::random::mt19937 rng;

    //ROS variables
    ros::NodeHandle nh_;

    //vehicle wheelbase
    float wheel_base_;
    float wheel_diameter_;
    float max_speed_;
    float max_steering_;

    //FastSLAM configuration parameters
    const int num_particles_;
    const int num_landmarks_;
    const int state_size_;
    const int lm_size_;
    const int num_particles_resampling_;
    float resampling_ratio_;

    //estimate variables
    Eigen::Vector3f xEst_;
    const int update_frequency_;

    //Initialise vector of particles
    std::vector<TestParticle> particles_;

    //SLAM phase switch
    bool lap_closure_detected_;

    //position and velocity odometric estimates
    Eigen::Vector3f pose_;

    //FastSLAM algorithm noise
    Eigen::Matrix2f R_; //linearized vehicle measurement noise
    Eigen::Matrix2f Q_; //linearized vehicle control noise

    nav_msgs::Path motion_model_only_path_;
    nav_msgs::Path slam_path_;
    visualization_msgs::MarkerArray features_markers_;
    visualization_msgs::MarkerArray particles_markers_;

    ros::Publisher motion_path_pub_;
    ros::Publisher slam_path_pub_;
    ros::Publisher particles_pub_;
    ros::Publisher features_pub_;

};

#endif
