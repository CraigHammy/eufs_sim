#include "slam_utils.hpp"
#include <Eigen/Dense>
#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <perception_pkg/Cone.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <cmath>
#include <vector>
#include <Eigen/Core>
#include <fstream>
#include <iostream>

/**
 * @brief Computes landmark EKF predicted observation, Jacobians wrt to landmark and robot locations and landmark innovation covariance
 * @param mean Eigen 3D vector describing the Mean of the landmark EKF 
 * @param sigma Eigen 3x3 matrix describing the covariance of the landmark EKF 
 * @param landmark A Landmark object
 * @param R Eigen 2x2 covariance matrix of measurement noise
 * @return Innovation variable which includes landmark EKF predicted observation, Jacobians wrt to landmark and robot locations and landmark innovation covariance
 */
Innovation computeInnovations(const Eigen::Vector3f& mean, const Eigen::Matrix3f& sigma, const Landmark& landmark, const Eigen::Matrix2f& R)
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

/**
 * @brief Retrieve measurement containing distance and angle to landmark
 * @param sensor_reading A Point message describing a sensor reading from the Velodyne
 * @return An Eigen 2d vector describing the distance and angle to the landmark
 */
Eigen::Vector2f getMeasurement(const geometry_msgs::Point::ConstPtr& observation)
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
    float yaw1 = transform.getRotation().getZ();
    float x1 = transform.getOrigin().getX();
    float y1 = transform.getOrigin().getY();
    transformation << cosf(yaw1), -sinf(yaw1), x1, sinf(yaw1), cosf(yaw1), y1, 0, 0, 1;

    //construct transformation matrix from Point data (velodyne to cone)
    Eigen::Matrix3f cone;
    float x2 = observation->x;
    float y2 = observation->y;
    float yaw2 = atan2f(y2, x2);
    cone << cosf(yaw2), -sinf(yaw2), x2, sinf(yaw2), cosf(yaw2), y2, 0, 0, 1;

    //multiply the matrix to get transformation matrix of landmark location from base_link frame of reference
    Eigen::Matrix3f transformed_cone(transformation * cone);
    float final_x = transformed_cone(0, 2);
    float final_y = transformed_cone(1, 2);
    float final_yaw = atan2f(final_y, final_x);
    final_yaw = angleWrap(final_yaw);

    //construct 2d vector made of euclidean distance and angle to landmark
    Eigen::Vector2f z;
    float distance = sqrtf(powf(final_x, 2) + powf(final_y, 2));
    z << distance, final_yaw; 

    return z;
}

/**
 * @brief Writes data to csv file
 * @param filepath The file path name where the csv file will be saved
 * @param num_columns Number of columns the csv file will have 
 * @param input Vector which stores the data to put in the csv file 
 */
void writeToCSV(const std::string& filepath, int num_columns, std::vector<float> input)
{
    //retrieve number of columns from first entry 
    //std::cout << input.size() << std::endl;
    //int columns = int(input.at(0));
    //input.erase(input.begin());

    //creare CSV file stream 
    std::ofstream csv_file;
    csv_file.open(filepath.c_str());

    //csv_file << "Ground-Truth x (m)" << "," << "Ground-Truth y (m)" << "," << "Prediction x (m)" << "," << 
    //            "Prediction y (m)" << "," << "Time (s)" << "," << "Error (m)" << std::endl;

    for (int i = 0; i != input.size(); ++i)
    {
        /*
        int k = i*num_columns;
        
        for (int j = 0; j !=num_columns; ++j)
        {
            csv_file << input[k+j]; 
            ROS_INFO("index: %d, value: %f", k+j, input[k+j]);
            if (j == num_columns-1)
            {
                csv_file << std::endl;
            }
            else
            {
                csv_file << ",";
            }
        }*/
        
        if (num_columns == 6)
        {   
            int j = i*num_columns;
            if ((input[((i-1)*num_columns) + 4] > 50.0) && (fabs(input[((i-1)*num_columns) + 4] - input[j+4]) > 2.0))
                break;
            csv_file << input[j+0] << "," << input[j+1] << "," << input[j+2] << "," << input[j+3] << ","
                << input[j+4] << "," << input[j+5] << std::endl;
        }
        else if (num_columns == 4)
        {   
            int j = i*num_columns;
            csv_file << input[j+0] << "," << input[j+1] << "," << input[j+2] << "," << input[j+3] << std::endl;
        }
        else if (num_columns == 2)
        {   
            int j = i*num_columns;
            csv_file << input[j+0] << "," << input[j+1] << std::endl;
        }
        
    }
    csv_file.close();
}