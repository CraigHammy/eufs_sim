#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Odometry.h>

/*
In relation to a body the standard is:

x forward
y left
z up
*/

class WorldFrameBroadcaster
{
public:
    /**
     * @brief Constructor for the global frame broadcaster class
     */
    WorldFrameBroadcaster(ros::NodeHandle* nh): nh_(*nh), private_nh_("~") { initialise(); }

    /**
     * @brief Initialises the subscriber to the topic that publishes the current SLAM estimate of the robot inside the map  
     */
    void initialise()
    {
        private_nh_.getParam("pose_topic", name);
        std::cout << name << std::endl;
        slam_sub_ = nh_.subscribe<nav_msgs::Odometry>(name, 1, boost::bind(&WorldFrameBroadcaster::slam_callback, this, _1));
    }

    /**
     * @brief Callback for Odometry messages
     * @param An Odometry message 
     */
    void slam_callback(const nav_msgs::Odometry::ConstPtr& msg)
    {
        transformStamped_.header.frame_id = "track";
        transformStamped_.child_frame_id = "base_footprint";
        transformStamped_.header.stamp = ros::Time::now();

        transformStamped_.transform.translation.x = msg->pose.pose.position.x;
        transformStamped_.transform.translation.y = msg->pose.pose.position.y;
        transformStamped_.transform.translation.z = msg->pose.pose.position.z;
        
        geometry_msgs::Quaternion orient = msg->pose.pose.orientation;
        transformStamped_.transform.rotation = orient;

        tfb_.sendTransform(transformStamped_);
        //ROS_INFO("x: %f, y: %f, yaw:%f", msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.orientation.z);
    }

private:
    //ROS helper variables 
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
    ros::Subscriber slam_sub_;
    tf2_ros::TransformBroadcaster tfb_;
    geometry_msgs::TransformStamped transformStamped_;
    std::string name;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "track_frame_broadcaster_node");
    ros::NodeHandle nh;

    //create frame broadcaster class and keep it open until the node gets killed
    WorldFrameBroadcaster tbc(&nh);
    ros::spin();
    return 0;
}