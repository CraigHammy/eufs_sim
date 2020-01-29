#ifndef UTILITIES_HEADER
#define UTILITIES_HEADER

#include <Eigen/Dense>
#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <perception_pkg/Cone.h>
#include <tf/transform_listener.h>
#include "particle.hpp"
#include <cmath>

enum SLAM_PHASE {
    MAP_BUILDING,
    LOCALIZATION
};

struct RobotPose
{
    //odom variables
    Eigen::Vector3f odom_pose_;
    Eigen::Vector3f odom_twist_;
};

struct RobotCommand
{
    //linear velocity and steering angle
    float speed;
    float steering_angle;
};

struct Landmark
{
    Eigen::Vector2f mu_;
    Eigen::Matrix2f sigma_;
    std::string colour_;
};

static const int NEW_LANDMARK_THRESH = 0.001;

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
    int count = 0;
    for(auto a: weights)
    {
        if (count == 0)
        {
            cumulative_weights(count) = a;
        }
        else 
        {
            cumulative_weights(count) = cumulative_weights(count-1) + a;
        }
    }
    return cumulative_weights;
}

/**
 * @brief Retrieve measurement containing distance and angle to landmark
 * @param sensor_reading A Point message describing a sensor reading from the Velodyne
 * @return An Eigen 2d vector describing the distance and angle to the landmark
 */
Eigen::Vector2f getMeasurement(const geometry_msgs::Point::ConstPtr& observation, float angle)
{
    //get transformation from base link to velodyne sensor 
    static tf::TransformListener listener;
    tf::StampedTransform transform;

    //create expected link  
    std::string velodyne_link = "/velodyne";
    
    //block until transform becomes available or until timeout has been reached 
    ros::Time now = ros::Time::now();
    try{
      listener.lookupTransform("/base_link", velodyne_link, now, transform);
    }
    catch (tf::TransformException e){
        ROS_ERROR("%s", e.what());
        ros::Duration(1.0).sleep();
    }
  
    //construct transformation matrix from Transform data 
    Eigen::Matrix3f transformation;
    float yaw1 = transform.getRotation().z;
    float x1 = transform.getOrigin().x;
    float y1 = transform.getOrigin().y;
    transformation << cosf(yaw1), -sinf(yaw1), x1, sinf(yaw1), cosf(yaw1), y1, 0, 0, 1;

    //construct transformation matrix from Point data
    Eigen::Matrix3f cone;
    float x2 = observation->x;
    float y2 = observation->y;
    float yaw2 = atan2f(y2, x2);
    cone << cosf(yaw2), -sinf(yaw2), x2, sinf(yaw2), cosf(yaw2), y2, 0, 0, 1;

    //multiply the matrix to get transformation matrix of landmark location from base_link frame of reference
    Eigen::Matrix3f transformed_cone(transformation * cone);
    float final_x = transformed_cone(0, 2);
    float final_y = transformed_cone(1, 2);
    //WHY AM I ADDING THE CURRENT ROBOT ANGLE TO IT ? EITHER MAKE IT ONLY RESPECT TO CURRENT ROBOT POSE OR MAP 
    float final_yaw = acosf(transformed_cone(0, 0) + angle);
    final_yaw = angleWrap(final_yaw);

    //construct 2d vector made of euclidean distance and angle to landmark
    Eigen::Vector2f z;
    float distance = sqrtf(powf(final_x, 2) + powf(final_y, 2));
    z << distance, final_yaw; 

    return z;
}

#endif
