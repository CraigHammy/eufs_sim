#include <ros/ros.h>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <vector>
#include "montecarlo_localisation.hpp"
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include "slam_utils.hpp"
#include <limits>
#include <cmath>

/**
 * @brief Initialises parameters and variables needed for the Extended Kalman Filter
 */
void MCL::initialise()
{
    private_nh_->getParam("scan_min_range", min_range_);
    private_nh_->getParam("scan_max_range", max_range_);
    private_nh_->getParam("scan_min_angle", min_angle_);
    private_nh_->getParam("scan_max_angle", max_angle_);
    private_nh_->getParam("scan_angle_increment", angle_increment_);

    filtered_scan_pub_ = private_nh_->advertise<visualization_msgs::MarkerArray>("/filtered_scan", 1);
    predicted_scan_pub_ = private_nh_->advertise<visualization_msgs::MarkerArray>("/predicted_scan", 1);
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
    filtered_scan_pub_.publish(filtered_scan_);
    
    std::vector<float> z_pred(predictObservation(xEst, scan.size()).ranges);
    predicted_scan_pub_.publish(predicted_scan_);
/*
    //calculate the difference 
    Eigen::VectorXf differences;
    for(int i = 0; i != z.size(); ++i)
    {
        float difference = (z[i] - z_pred[i]);
        differences(i) = difference;
    }

    //squared norm of the difference
    float error = powf(differences.norm(), 2); 
    return expf(error);*/
}

/**
 * @brief Uses a transform listener to return the transformation matrix from the base_footprint to the velodyne frame
 * @return Eigen 3D transformation matrix 
 */
Eigen::Matrix3f MCL::getBaseToVelodyneTF()
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

    for(int i = 1; i != scan.size(); ++i)
    {
        float current_scan, previous_scan;
        if (std::isinf(scan[i]))
            current_scan = 11.0;
        else 
            current_scan = scan[i];
        if (std::isinf(scan[i-1]))
            previous_scan = 11.0;
        else
            previous_scan = scan[i-1];
        
        //if scan found, there will be other ones belonging to same cone
        if ((current_scan < max_range_) && (current_scan > min_range_))
        {
            if (temp.size() == 0)
            {
                //add all intensity values belonging to same cone from left to right
                temp.push_back(current_scan);
                continue;
            }
            else if ((temp.size() > 0) && (fabs(current_scan - previous_scan) < 0.4))
            {
                //add all intensity values belonging to same cone from left to right
                temp.push_back(current_scan);
                continue;
            }
        } 

        if (temp.size() == 0)
            continue;
        
        //add to filtered scan, the middle point from all the individual cone intensity scans
        float dist = temp[int(temp.size() / 2)];
        float yaw = min_angle_ + (i - int(temp.size() / 2)) * angle_increment_;

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
        float final_yaw = atan2f(final_y, final_x);
        final_yaw = angleWrap(final_yaw);

        //euclidean distance and yaw from robot to cone
        float final_dist = sqrtf(powf(final_x, 2) + powf(final_y, 2));
        ranges.push_back(final_dist);
        angles.push_back(final_yaw);

        visualization_msgs::Marker marker;
        marker.header.frame_id = "base_footprint";
        marker.header.stamp = ros::Time::now();
        marker.id = ranges.size() - 1;
        marker.type = visualization_msgs::Marker::CUBE;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.x = final_dist*cosf(final_yaw);
        marker.pose.position.y = final_dist*sinf(final_yaw);
        marker.pose.position.z = 0;
        marker.scale.x = 0.07;
        marker.scale.y = 0.07;
        marker.scale.z = 0.07;
        marker.color.r = 0.0f;
        marker.color.g = 0.0f;
        marker.color.b = 1.0f;
        marker.color.a = 1.0;
        filtered_scan_.markers.push_back(marker);

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
    int counter = 0;
    std::vector<float> ranges, angles;
    std::vector<int> js;
    for(int j = 0; j != map_.data.size(); ++j)
    {
        if (map_.data[j] < 0 || map_.data[j] > 25)
        {
            js.push_back(j);
            int cellY = j % map_.info.width;
            int cellX = j / map_.info.width;
            float x = (cellX * map_.info.resolution) + map_.info.origin.position.x;
            float y = (cellY * map_.info.resolution) + map_.info.origin.position.y;
            //ROS_WARN("POSE X: %f, Y: %f", x, y);
        }
    }
    ROS_WARN("hello3");
    for(int i = 0; i < num_intensities; ++i)
    {
        float curr_yaw = xEst(2) + (min_angle_ + i * angle_increment_);
        float dist = min_range_;
        ROS_WARN("current angle: %f", curr_yaw);
        //ray tracing for each angle
        while (dist != max_range_)
        {
            //ROS_WARN("hello5");
            float x = xEst(0) + dist * cosf(curr_yaw);
            float y = xEst(1) + dist * sinf(curr_yaw);

            if (x > max_range_ || x < 0 || y > max_range_ || y < -max_range_)
                break;
            
            if (sqrtf(powf(x, 2) + powf(y, 2)) >= max_range_)
                break;
            
            //get cell number from pose
            int cell_x = (x / map_.info.resolution) + (map_.info.width / 2);
            int cell_y = (y / map_.info.resolution) + (map_.info.height / 2);

            //ROS_INFO("POSE X: %f, Y: %f", x, y);
            //ROS_INFO("CELL X: %d, Y: %d", cell_x, cell_y);

            //get occupancy grid map array index 
            int idx = map_.info.width * cell_x + cell_y;

            std::vector<int>::iterator it;
            it = std::find(js.begin(), js.end(), idx);
            if (it != js.end())
                std::cout << "Element found" << std::endl;

            //check if cell is not free
            if (map_.data[idx] < 0 || map_.data[idx] > 25)
            {
                ROS_WARN("hello6");
                float final_dist, final_yaw;
                //check if the previous is the neighbouring cell
                if (fabs(curr_yaw - prev_yaw) == angle_increment_)
                {
                    ROS_WARN("hello7");
                    final_dist = (dist + prev_dist) / 2.0;
                    final_yaw = (curr_yaw + prev_yaw) / 2.0;
                } 
                else 
                {
                    ROS_WARN("hello8");
                    final_dist = dist;
                    final_yaw = curr_yaw;
                }
                ROS_WARN("hello9");
                visualization_msgs::Marker marker;
                marker.header.frame_id = "track";
                marker.header.stamp = ros::Time::now();
                marker.id = counter;
                marker.type = visualization_msgs::Marker::CUBE;
                marker.action = visualization_msgs::Marker::ADD;
                marker.pose.position.x = x;
                marker.pose.position.y = y;
                marker.pose.position.z = 0;
                marker.scale.x = 0.07;
                marker.scale.y = 0.07;
                marker.scale.z = 0.07;
                marker.color.r = 0.0f;
                marker.color.g = 1.0f;
                marker.color.b = 0.0f;
                marker.color.a = 1.0;
                predicted_scan_.markers.push_back(marker);
                ++counter;

                ROS_WARN("hello10");
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