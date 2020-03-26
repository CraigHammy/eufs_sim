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
 * @brief  Monte Carlo Localisation weight calculation using real and predicted observation difference
 * @param xEst Eigen 3D vector describing the input from the EKF (prediction step)
 * @param scan Intensity array of the LaserScan message
 * @return The weight of the particle 
 */
float MCL::amcl_estimation_step(const Eigen::Vector3f& xEst, const std::vector<float>& scan)
{
    //do it for every particle 
    std::vector<float> z(filterScan(xEst, scan).ranges);
    std::vector<float> z_pred(predictObservation(xEst, scan.size()).ranges);

    //calculate the difference 
    Eigen::VectorXf differences;
    for(int i = 0; i != z.size(); ++i)
    {
        float difference = (z[i] - z_pred[i]);
        differences(i) = difference;
    }

    //squared norm of the difference
    float error = powf(differences.norm(), 2); 
    return expf(error);
}

/**
 * @brief Uses a transform listener to return the transformation matrix from the base_footprint to the velodyne frame
 * @return Eigen 3D transformation matrix 
 */
Eigen::Matrix3f getBaseToVelodyneTF()
{
    //get transformation from base link to velodyne sensor 
    static tf::TransformListener listener;
    tf::StampedTransform transform;

    //create expected link  
    std::string velodyne_link = "velodyne";
    
    //block until transform becomes available or until timeout has been reached 
    ros::Time now = ros::Time::now();
    try{
        listener.waitForTransform("base_footprint", velodyne_link, now, ros::Duration(3.0));
        listener.lookupTransform("base_footprint", velodyne_link, now, transform);
    }
    catch (tf::TransformException e){
        ROS_ERROR("%s", e.what());
        ros::Duration(1.0).sleep();
    }
  
    //construct transformation matrix (base_link to velodyne) from Transform data 
    Eigen::Matrix3f transformation;
    float yaw = transform.getRotation().getZ();
    float x = transform.getOrigin().getX();
    float y = transform.getOrigin().getY();
    transformation << cosf(yaw), -sinf(yaw), x, sinf(yaw), cosf(yaw), y, 0, 0, 1;
    return transformation;
}

/**
 * @brief Filtering intensity array of LaserScan message to one point per cloud of laser detections
 * @param xEst Eigen 3D vector describing the input from the EKF (prediction step)
 * @param scan Intensity array of the LaserScan message
 * @return Filtered ranges and angles vector 
 */
Observations MCL::filterScan(const Eigen::Vector3f xEst, const std::vector<float>& scan)
{
    //get transform from base footprint to velodyne link, useful as scans are in the velodyne frame
    Eigen::Matrix3f t1(getBaseToVelodyneTF());

    //iterate over laser scan intensity message
    std::vector<float> ranges, angles;
    std::vector<float> temp;
    for(int i = 0; i != scan.size(); ++i)
    {
        //if scan found, there will be other ones belonging to same cone
        while ((scan[i] < max_range_) && (scan[i] > min_range_) && (abs(scan[i] - scan[i-1]) < 0.4))
        {
            //add all intensity values belonging to same cone from left to right
            temp.push_back(scan[i]);
        }
        
        //add to filtered scan, the middle point from all the individual cone intensity scans
        float dist = temp[int(temp.size() / 2)];
        float yaw = -min_angle_ + i * angle_increment_;

       //location of cone in x and y coordinates
        float x = dist * cosf(yaw);
        float y = dist * sinf(yaw);

        //create transformation matrix from velodyne to cone 
        Eigen::Matrix3f t2;
        t2 << cosf(yaw), -sinf(yaw), x, sinf(yaw), cosf(yaw), y, 0, 0, 1;

        //multiply with transform from base to velodyne to get values in correct frame (base footprint)
        Eigen::Matrix3f t3(t1 * t2);
        float final_x = t3(0, 2);
        float final_y = t3(1, 2);
        float final_yaw = asinf(t3(1, 0));
        final_yaw = angleWrap(final_yaw);

        //euclidean distance and yaw from robot to cone
        float final_dist = sqrtf(powf(final_x, 2) + powf(final_y, 2));
        ranges.push_back(final_dist);
        angles.push_back(final_yaw);

        //remove all elements for next iteration
        temp.clear();
    }

    Observations obs;
    obs.ranges = ranges;
    obs.angles = angles;
    return obs;
}

/**
 * @brief Based on landmark map and current robot position, predict positions of cones 
 * @param xEst Eigen 3D vector describing the input from the EKF (prediction step)
 * @param num_intensities The size of the scans intensity vector
 * @return Predicted ranges and angles vector 
 */
Observations MCL::predictObservation(const Eigen::Vector3f& xEst, int num_intensities)
{
    float prev_dist, prev_yaw;
    std::vector<float> ranges, angles;

    for(int i = 0; i < num_intensities; ++i)
    {
        float curr_yaw = xEst(2) + (-min_angle_ + i * angle_increment_);
        float dist = min_range_;

        //ray tracing for each angle
        while (dist != max_range_)
        {
            float x = xEst(0) + dist * cosf(curr_yaw);
            float y = xEst(1) + dist * sinf(curr_yaw);

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
                float final_dist, final_yaw;
                //check if the previous is the neighbouring cell
                if (fabs(curr_yaw - prev_yaw) == angle_increment_)
                {
                    final_dist = (dist + prev_dist) / 2.0;
                    final_yaw = (curr_yaw + prev_yaw) / 2.0;
                } 
                else 
                {
                    final_dist = dist;
                    final_yaw = curr_yaw;
                }
                ranges.push_back(final_dist);
                angles.push_back(final_yaw);

                prev_yaw = curr_yaw;
                prev_dist = dist;
                break;
            }
            dist += map_.info.resolution;
        }
    }
    Observations obs;
    obs.ranges = ranges;
    obs.angles = angles;
    return obs;  
}