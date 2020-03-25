#include <ros/ros.h>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <vector>
#include "montecarlo_localisation.hpp"
#include "slam_utils.hpp"
#include <limits>

/**
 * @brief Initialises parameters and variables needed for the Extended Kalman Filter
 */
void MCL::initialise()
{
    private_nh_->getParam("scan_min_range", min_range_);
    private_nh_->getParam("scan_max_range", max_range_);
    private_nh_->getParam("scan_min_angle", min_range_);
    private_nh_->getParam("scan_max_angle", max_range_);
    private_nh_->getParam("scan_angle_increment", angle_increment_);

}

/**
 * @brief Monte Carlo Localisation step: prediction and measurement update (correction)
 * @param xEst Eigen 3D vector describing the input from the EKF (prediction step)
 * @param scan Intensity array of the LaserScan message
 * @return Eigen 3D vector describing the pose of the robot after MCL prediction and measurement steps 
 */
Eigen::Vector3f MCL::amcl_estimation_step(Eigen::Vector3f& xEst, const std::vector<float>& scan)
{

    //INSTEAD OF GENERATING POSITIONS OF LANDMARKS GENERATE DISTANCE FROM ROBOT TO POSSIBLE LANDMARK 
    //do it for every particle 
    std::vector<Eigen::Vector2f> z(filterScan(xEst, scan));
    std::vector<Eigen::Vector2f> z_pred(predictObservation(xEst, scan.size()));

    //calculate the difference 
    std::vector<float> differences;
    for(int i = 0; i != z.size(); ++i)
    {
        float difference = (z[i] - z_pred[i]).norm();
        differences.push_back(difference);
    }
    //squared norm of that difference
    

}

/**
 * @brief Filtering intensity array of LaserScan message to one point per cloud of laser detections
 * @param xEst Eigen 3D vector describing the input from the EKF (prediction step)
 * @param scan Intensity array of the LaserScan message
 * @return Vector of Eigen 2D vectors each representing one point per cloud of close intensity values
 */
std::vector<Eigen::Vector2f> MCL::filterScan(const Eigen::Vector3f xEst, const std::vector<float>& scan)
{
    //iterate over laser scan intensity message
    std::vector<Eigen::Vector2f> scan_filtered;
    std::vector<float> temp;
    for(int i = 0; i != scan.size(); ++i)
    {
        //if scan found, there will be other ones belonging to same cone
        while ((scan[i] < max_range_) && (scan[i] > min_range_))
        {
            //add all intensity values belonging to same cone from left to right
            temp.push_back(scan[i]);
        }
        //add to filtered scan, the middle point from all the individual cone intensity scans
        float val = temp[int(temp.size() / 2)];
        float angle = -min_angle_ + i * angle_increment_;

        geometry_msgs::Point p;
        p.x = val * cosf(angle);
        p.y = val * sinf(angle);
        p.z = 0.0;
        boost::shared_ptr<geometry_msgs::Point> z_ptr(new geometry_msgs::Point(p));
        Eigen::Vector2f measurement(getMeasurement(z_ptr));

        //sine and cosine of yaw angle
        float s = sinf(angleWrap(xEst(2) + measurement(1)));
        float c = cosf(angleWrap(xEst(2) + measurement(1)));

        //x and y values of landmark respect to robot pose
        Eigen::Vector2f final;
        final << xEst(0) + measurement(0)*c, xEst(1) + measurement(0)*s;
        scan_filtered.push_back(final);

        //remove all elements for next iteration
        temp.clear();
    }
    return scan_filtered;
}

/**
 * @brief Resamples the particles based on the updated weights from the correction step
 */
void MCL::resampling()
{

}

/**
 * @brief
 * @param
 * @result
 */
void MCL::calculateWeight()
{

}

/**
 * @brief Based on landmark map and current robot position, predict positions of cones 
 * @param xEst Eigen 3D vector describing the input from the EKF (prediction step)
 * @param num_intensities The size of the scans intensity vector
 * @return Predicted vector of Eigen 2D vector values 
 */
std::vector<Eigen::Vector2f> MCL::predictObservation(const Eigen::Vector3f& xEst, int num_intensities)
{
    std::vector<Eigen::Vector2f> pred_z;

    for(int i = 0; i < num_intensities; ++i)
    {
        float curr_angle = xEst(2) + (-min_angle_ + i * angle_increment_);
        float dist = min_range_;

        //ray tracing for each angle
        while (dist != max_range_)
        {
            float x = xEst(0) + dist * cosf(curr_angle);
            float y = xEst(1) + dist * sinf(curr_angle);

            if (x > max_range_ || x < -max_range_ || y > max_range_ || y < 0)
                break;
            
            //get cell number from pose
            int cell_x = (x / map_.info.resolution) + (map_.info.width / 2);
            int cell_y = (y / map_.info.resolution) + (map_.info.height / 2);

            //get occupancy grid map array index 
            int idx = map_.info.width * cell_x + cell_y;

            //check if cell is not free
            if (map_.data[idx] < 0 || map_.data[idx] > 25)
            {
                Eigen::Vector2f position(x, y);
                pred_z.push_back(position);
                break;
            }
            dist += map_.info.resolution;
        }
    }
    return pred_z;   
}

/**
 * @brief 
 * @param 
 * @result
 */
void MCL::measurementUpdate()
{

}