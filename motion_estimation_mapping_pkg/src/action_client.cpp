#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <motion_estimation_mapping_pkg/FastSlamAction.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "slam_client");

    ros::NodeHandle nh;// create the action client
    actionlib::SimpleActionClient<motion_estimation_mapping_pkg::FastSlamAction> ac("fastslam", true);

    ROS_INFO("Waiting for action server to start.");
    ac.waitForServer(); //will wait for infinite time
    ROS_INFO("Action server started, sending goal.");

    // send a goal to the action
    motion_estimation_mapping_pkg::FastSlamGoal goal;
    ac.sendGoal(goal);

    //wait for the action to return
    ac.waitForResult();
    actionlib::SimpleClientGoalState state = ac.getState();
    ROS_INFO("Action finished: %s",state.toString().c_str());
    
    return 0;
}