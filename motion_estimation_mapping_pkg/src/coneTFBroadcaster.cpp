#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <perception_pkg/Cone.h>

/*
In relation to a body the standard is:

x forward
y left
z up

Suffix Frames
In the case of cameras, there is often a second frame defined with a "_optical" suffix. This uses a slightly different convention:

z forward
x right
y down
*/

/**
 * @brief Broadcasts the TF of the cone in a personalized format 
 * @param msg A Cone message
 */
void conePoseCallback(const perception_pkg::ConeConstPtr& msg)
{
    static tf::TransformBroadcaster br;
    std::string colour = msg->colour;
    int id = msg->header.seq;
    tf::Transform transform;
    transform.setOrigin(tf::Vector3(msg->location.x, msg->location.y, msg->location.z));
    //if LiDAR and camera frames x, y and z are different create a method to transform them to an universal transform 
    tf::Quaternion q;
    q.setRPY(0, 0, 0);
    transform.setRotation(q);
    //format of cone transform is {"cone"_id_colour} where id the the detection number from the start 
    std::string link_name = "/cone" + '_' + id + '_' + colour;
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), msg->child_frame_id, link_name));
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "cone_tf_broadcaster node");
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe("/cones", 1, &conePoseCallback);
    ros::spin();
    return 0;
}