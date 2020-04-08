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
    std::cout << xEst << std::endl;
    std::cout << scan.size() << std::endl;
    std::vector<float> z(filterScan(xEst, scan).ranges);
    ROS_INFO("ciao");
    std::vector<float> z_pred(predictObservation(xEst, scan.size()).ranges);
    ROS_INFO("ciao1");
    
    //filtered_scan_pub_.publish(filtered_scan_);
    //predicted_scan_pub_.publish(predicted_scan_);

    /*ROS_WARN("filtered range size is %d, predicted scan size is %d", int(z.size()), int(z_pred.size()));
    float average;
    if (z.size() == z_pred.size())
    {
        float sum = 0;
        for (int i = 0; i != z.size(); ++i)
        {
            sum += fabs(z_pred[i] - z[i]);
        }
        average = sum / z.size();
        ROS_WARN("average difference is %f", average);
    }*/

    filtered_scan_.markers.clear();
    predicted_scan_.markers.clear();
    ROS_INFO("ciao2");

    //calculate the difference 
    Eigen::VectorXf differences(z.size());
    for(int i = 0; i != z.size(); ++i)
    {
        ROS_INFO("ciao3");
        float difference = (z[i] - z_pred[i]);
        differences(i) = difference;
    }
    ROS_INFO("ciao4");
    //squared norm of the difference
    float error = powf(differences.norm(), 2); 
    return expf(error);
}

/**
 * @brief Uses a transform listener to return the transformation matrix from the base to the velodyne frames
 * @return Eigen 3D transformation matrix 
 */
Eigen::Matrix3f MCL::getBaseToVelodyneTF()
{
    //get transformation from base link to velodyne sensor 
    static tf::TransformListener listener;
    tf::StampedTransform transform;
    
    //block until transform becomes available or until timeout has been reached 
    ros::Time now = ros::Time::now();
    try{
        listener.waitForTransform("base_footprint", "velodyne", now, ros::Duration(3.0));
        listener.lookupTransform("base_footprint", "velodyne", now, transform);
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
 * @brief Subsamples the ranges array of the laser scan message 
 * @param scan Ranges array of the LaserScan message
 * @param step Values to skip in the scan array for every value added
 * @return Subsampled array 
 */
std::vector<float> subsampleRanges(const std::vector<float>& scan, float step)
{
    std::vector<float> subsampled;
    for(int i = 0; i <= scan.size(); i+step)
    {
        subsampled.push_back(scan[i]);
    }
    return subsampled;
}

/**
 * @brief Filtering intensity array of LaserScan message to one point per cloud of laser detections
 * @param xEst Eigen 3D vector describing the input from the EKF (prediction step)
 * @param scan Intensity array of the LaserScan message
 * @return Filtered ranges and angles vector 
 */
Observations MCL::filterScan(const Eigen::Vector3f xEst, const std::vector<float>& scan)
{
    //iterate over laser scan intensity message
    std::vector<float> ranges, angles;
    std::vector<float> temp;

    //ROS_WARN("filtering..");

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
        //ROS_WARN("filtered scan range:%f", dist);
        float yaw = min_angle_ + (i - int(temp.size() / 2)) * angle_increment_;

       //location of cone in x and y coordinates
        float x = dist * cosf(yaw);
        float y = dist * sinf(yaw);

       //ROS_WARN("adding marker");

        ranges.push_back(dist);
        angles.push_back(yaw);
/*
        visualization_msgs::Marker marker;
        marker.header.frame_id = "velodyne";
        marker.header.stamp = ros::Time::now();
        marker.id = ranges.size() - 1;
        marker.type = visualization_msgs::Marker::CUBE;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.x = x;
        marker.pose.position.y = y;
        marker.pose.position.z = 0;
        marker.scale.x = 0.07;
        marker.scale.y = 0.07;
        marker.scale.z = 0.07;
        marker.color.r = 0.0f;
        marker.color.g = 0.0f;
        marker.color.b = 1.0f;
        marker.color.a = 1.0;
        filtered_scan_.markers.push_back(marker);
*/
        //remove all elements for next iteration
        temp.clear();
    }
    //ROS_WARN("number of filtered scan markers is %d", int(filtered_scan_.markers.size()));
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
    Eigen::Vector2f prev_pose;
    int counter = 0;
    int counter2 =0;
    std::vector<float> ranges, angles;

    
    Eigen::Matrix3f t(getBaseToVelodyneTF());
    ROS_INFO("hola");
    Eigen::Matrix3f current_pose;
    current_pose << cosf(xEst(2)), -sinf(xEst(2)), xEst(0), sinf(xEst(2)), cosf(xEst(2)), xEst(1), 0, 0, 1;
    ROS_INFO("hola2");
    Eigen::Matrix3f velodyne_pose(current_pose*t);
    float start_x = velodyne_pose(0, 2);
    float start_y = velodyne_pose(1, 2);
    float start_yaw = atan2f(start_y, start_x);
    
    std::vector<float> same_cone, same_cone2;

    for(int i = 0; i < num_intensities; ++i)
    {
        //ROS_INFO("hola3");
        if (i == num_intensities - 1)
        {
            float final_dist = same_cone[int(same_cone.size() / 2)];
            float final_yaw = same_cone2[int(same_cone2.size() / 2)];

            same_cone.clear();
            same_cone2.clear();
                    
            float vis_x = final_dist * cosf(final_yaw);
            float vis_y = final_dist * sinf(final_yaw);

            /*visualization_msgs::Marker marker;
            marker.header.frame_id = "velodyne";
            marker.header.stamp = ros::Time::now();
            marker.id = counter;
            marker.type = visualization_msgs::Marker::CUBE;
            marker.action = visualization_msgs::Marker::ADD;
            marker.pose.position.x = vis_x;
            marker.pose.position.y = vis_y;
            marker.pose.position.z = 0;
            marker.scale.x = 0.1;
            marker.scale.y = 0.1;
            marker.scale.z = 0.1;
            marker.color.r = 0.0f;
            marker.color.g = 1.0f;
            marker.color.b = 0.0f;
            marker.color.a = 1.0;
            predicted_scan_.markers.push_back(marker);
            ++counter;*/

            ranges.push_back(final_dist);
            angles.push_back(final_yaw);
        }
         
        float curr_yaw = start_yaw + (min_angle_ + i * angle_increment_);
        //ROS_INFO("starting distance: %f", min_range_);
        //ROS_INFO("current location x: %f, y: %f", xEst(0), xEst(1));
        float dist = min_range_;
        float max = max_range_;
        //std::cout << max_range_ << std::endl;
        
        //ray tracing for each angle
        while (dist <= max)
        {
            //ROS_WARN("hello5");
            //ROS_INFO("hola4");
            //std::cout << dist << std::endl;
            //std::cout << curr_yaw << std::endl;
            float dx = dist * cosf(curr_yaw);
            float dy = dist * sinf(curr_yaw);
            float x = start_x + dx;
            float y = start_y + dy;
            Eigen::Vector2f curr_pose(dx, dy);
            
            if (dx > max_range_ || dx < 0 || dy > max_range_ || dy < -max_range_)
            {
                ROS_WARN("dx: %f, dy: %f, max_range: %f", dx, dy, max_range_);
                break;
            }

            if (sqrtf(powf(dx, 2) + powf(dy, 2)) >= max_range_)
            {
                ROS_INFO("ciao");
                ROS_WARN("Euclidean dist: %f, max_range: %f", sqrtf(powf(dx, 2) + powf(dy, 2)), max_range_);
                break;
            }

            //get cell number from pose
            int cell_x = (x / map_.info.resolution) + (map_.info.width / 2);
            int cell_y = (y / map_.info.resolution) + (map_.info.height / 2);

            //get occupancy grid map array index 
            int idx = map_.info.width * cell_x + cell_y;
            
            counter = 0;
            //check if cell is not free
            if (map_.data[idx] < 0 || map_.data[idx] > 25)
            {
                /*visualization_msgs::Marker marker1;
                marker1.header.frame_id = "velodyne";
                marker1.header.stamp = ros::Time::now();
                marker1.id = counter2;
                marker1.type = visualization_msgs::Marker::CUBE;
                marker1.action = visualization_msgs::Marker::ADD;
                marker1.pose.position.x = dx;
                marker1.pose.position.y = dy;
                marker1.pose.position.z = 0;
                marker1.scale.x = 0.07;
                marker1.scale.y = 0.07;
                marker1.scale.z = 0.07;
                marker1.color.r = 0.0f;
                marker1.color.g = 0.0f;
                marker1.color.b = 1.0f;
                marker1.color.a = 1.0;
                filtered_scan_.markers.push_back(marker1);
                ++counter2;*/

                float final_dist, final_yaw;
                //check if the previous is the neighbouring cell
                if (same_cone.size() == 0)
                {
                    same_cone.push_back(dist);
                    same_cone2.push_back(curr_yaw);
                    
                    prev_yaw = curr_yaw;
                    prev_dist = dist;
                    prev_pose = curr_pose;
                    dist += map_.info.resolution;
                    continue;
                }
                else if ((same_cone.size() > 0) && ((curr_pose - prev_pose).norm() < 0.6))
                {
                    same_cone.push_back(dist);
                    same_cone2.push_back(curr_yaw);

                    prev_yaw = curr_yaw;
                    prev_dist = dist;
                    prev_pose = curr_pose;
                    dist += map_.info.resolution;
                    continue;
                } 
                else if ((same_cone.size() > 0) && (((curr_pose - prev_pose).norm() > 0.6)))
                {
                    if (same_cone.size() == 1)
                    {
                        final_dist = same_cone.at(0);
                        final_yaw = same_cone2.at(0);
                        same_cone.clear();
                        same_cone2.clear();

                        same_cone.push_back(dist);
                        same_cone2.push_back(curr_yaw);
                    }
                    else if (same_cone.size() > 1)
                    {
                        final_dist = same_cone[int(same_cone.size() / 2)];
                        final_yaw = same_cone2[int(same_cone2.size() / 2)];
                        same_cone.clear();
                        same_cone2.clear();

                        same_cone.push_back(dist);
                        same_cone2.push_back(curr_yaw);
                    }
                }

                float vis_x = final_dist * cosf(final_yaw);
                float vis_y = final_dist * sinf(final_yaw);

                visualization_msgs::Marker marker;
                marker.header.frame_id = "velodyne";
                marker.header.stamp = ros::Time::now();
                marker.id = counter;
                marker.type = visualization_msgs::Marker::CUBE;
                marker.action = visualization_msgs::Marker::ADD;
                marker.pose.position.x = vis_x;
                marker.pose.position.y = vis_y;
                marker.pose.position.z = 0;
                marker.scale.x = 0.1;
                marker.scale.y = 0.1;
                marker.scale.z = 0.1;
                marker.color.r = 0.0f;
                marker.color.g = 1.0f;
                marker.color.b = 0.0f;
                marker.color.a = 1.0;
                predicted_scan_.markers.push_back(marker);
                ++counter;

                ranges.push_back(final_dist);
                angles.push_back(final_yaw);
                prev_yaw = curr_yaw;
                prev_dist = dist;
                prev_pose = curr_pose;
            }
            dist += map_.info.resolution;
        }
    }
    Observations obs;
    obs.ranges = ranges;
    obs.angles = angles;
    return obs;  
}