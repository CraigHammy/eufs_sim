#ifndef UTILITIES_HEADER2
#define UTILITIES_HEADER2

#include <Eigen/Dense>
#include <cmath>
#include <vector>
#include <Eigen/Core>

enum TEST_SLAM_PHASE {
    MAP_BUILDING,
    LOCALIZATION
};

struct TestLandmark
{
    Eigen::Vector2f mu_;
    Eigen::Matrix2f sigma_;
    std::string colour_;
};

struct ConeLocation 
{
    float x;
    float y;
    float z;
};

struct Innovation
{
    Eigen::Vector2f predicted_observation;
    Eigen::Matrix<float, 2, 3> vehicle_jacobian;
    Eigen::Matrix2f feature_jacobian;
    Eigen::Matrix2f innovation_covariance;
};

struct DataAssociation
{
    Eigen::Vector2f measurement;
    int landmark_id;
};

static const float NEW_LANDMARK_THRESH = 0.001;

static const float SAME_LANDMARK_THRESH = 0.50;

/**
 * @brief
 * @param
 * @return
 */
Innovation computeInnovations(const Eigen::Vector3f& mean, const Eigen::Matrix3f& sigma, const TestLandmark& landmark, const Eigen::Matrix2f& R)
{
    float dx = landmark.mu_(0) - mean(0);
    float dy = landmark.mu_(1) - mean(1);
    float d2 = powf(dx, 2) + powf(dy, 2);
    float d = sqrtf(d2);

    //Jacobian with respect to vehicle state
    Eigen::Matrix<float, 2, 3> Hv;
    Hv << -dx / d, -dy / d, 0, dy / d2, -dx / d2, -1;

    //Jacobian with respect to landmark state
    Eigen::Matrix2f Hl;
    Hl << dx / d, dy / d, -dy / d2, dx / d2;

    //measurement in the form of range and bearing to landmark from robot pose
    float theta = atan2f(dy, dx) - mean(2);
    theta = theta - 2* M_PI * floorf((theta + M_PI) / (2 * M_PI));

    Eigen::Vector2f zt;
    zt << d, theta;

    //innovation covariance of observed landmark
    Eigen::Matrix2f lm_innov_cov(Hl * landmark.sigma_ * Hl.transpose() + R);
    Innovation innovation = {zt, Hv, Hl, lm_innov_cov};
    return innovation;
}

/**
 * @brief Wraps the angle between -180 and +180 in radians
 * @param angle Angle value as float
 * @return Wrapped angle in radians 
 */
float angleWrap(float angle)
{
    return (angle - 2* M_PI * floorf((angle + M_PI) / (2 * M_PI)));
}

/**
 * @brief Generates Eigen array of cumulative sums 
 * @param weights Eigen array of floats
 * @return Resulting Eigen array of cumulative weights 
 */
Eigen::ArrayXf cumulativeSum(Eigen::ArrayXf weights)
{   
    Eigen::ArrayXf cumulative_weights(weights.size());
    for(int i = 0; i != weights.size(); ++i)
    {
        if (i == 0)
        {
            cumulative_weights(i) = weights(i);
        }
        else 
        {
            cumulative_weights(i) = cumulative_weights(i-1) + weights(i);
        }
    }
    return cumulative_weights;
}

#endif
