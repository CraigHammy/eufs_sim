#include <ros/ros.h>
#include <Eigen/Core> 
#include <Eigen/Dense>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/PoseStamped.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h> //fix dependency on move_base etc.

/* SETTINGS - Set parameters for the mapping lap here */
float waypointsMinDistance = 0.8;    // What minimum distance needs to be between two waypoints for the goal to be set to the new waypoint

//Simplify the big call
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
geometry_msgs::PoseStamped waypoint;    // The midpoint location to listen to

// Callback to get waypoints info
void waypointsCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
  waypoint = *msg;
  }

// Transform from one frame to another - need from velodyne to track (credits: @Valerio)
Eigen::Vector3f transform(float x, float y) {
  //get transformation from track frame to velodyne sensor 
    static tf::TransformListener listener;
    tf::StampedTransform transform;

    //create expected link  
    std::string velodyne_link = "velodyne";
    
    //block until transform becomes available or until timeout has been reached 
    ros::Time now = ros::Time::now();
    try{
        listener.waitForTransform("track", velodyne_link, now, ros::Duration(3.0));
        listener.lookupTransform("track", velodyne_link, now, transform);
    }
    catch (tf::TransformException e){
        ROS_ERROR("%s", e.what());
        ros::Duration(1.0).sleep();
    }
  
    //construct transformation matrix (track to velodyne) from Transform data 
    Eigen::Matrix3f transformation;
    float yaw1 = transform.getRotation().getZ();
    float x1 = transform.getOrigin().getX();
    float y1 = transform.getOrigin().getY();
    transformation << cosf(yaw1), -sinf(yaw1), x1, sinf(yaw1), cosf(yaw1), y1, 0, 0, 1;

    //construct transformation matrix from Point data (velodyne to cone)
    Eigen::Matrix3f cone;
    float x2 = x; //The position of the x and y of our Cone's Pose /!
    float y2 = y;
    float yaw2 = atan2f(y2, x2);
    cone << cosf(yaw2), -sinf(yaw2), x2, sinf(yaw2), cosf(yaw2), y2, 0, 0, 1;

    //multiply the matrix to get transformation matrix of landmark location from base_link frame of reference
    Eigen::Matrix3f transformed_cone(transformation * cone);
    float final_x = transformed_cone(0, 2);
    float final_y = transformed_cone(1, 2);
    float final_yaw = atan2f(final_y, final_x);

    Eigen::Vector3f output(final_x, final_y, final_yaw);
    return output;
}

// Method that returns the Euclidian distance between two points
float euclidianDifference(float x, float x2, float y, float y2) {return sqrt( pow(x-x2,2) + pow(y-y2,2) );}

// Method that sets a goal for the robot
move_base_msgs::MoveBaseGoal setGoal(float x, float y) {
  move_base_msgs::MoveBaseGoal goal;
  goal.target_pose.header.frame_id = "track";
  goal.target_pose.header.stamp = ros::Time::now();

  Eigen::Vector3f coords = transform(x,y);      // Get transformed coordinates

  goal.target_pose.pose.position.x = coords(0);
  goal.target_pose.pose.position.y = coords(1);
  goal.target_pose.pose.position.z = 0;

  goal.target_pose.pose.orientation.x = 0.0;
  goal.target_pose.pose.orientation.y = 0.0;
  goal.target_pose.pose.orientation.z = 0.0;
  goal.target_pose.pose.orientation.w = 1.0;

  return goal;
}

// Main loop
int main(int argc, char** argv) {
  ros::init(argc, argv, "mapping_lap");
  ros::NodeHandle n;

  // Tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);

  // Wait for the action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))) {
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  // Init goal var
  move_base_msgs::MoveBaseGoal goal;

  // Create a subscriber to listen to the topic /waypoints
  ros::Subscriber sub = n.subscribe("waypoints", 1000, waypointsCallback);
  

  geometry_msgs::PoseStamped temp = waypoint;                     // Get first waypoint
  goal = setGoal(waypoint.pose.position.x, waypoint.pose.position.y);    // Set first goal to first waypoint
  ROS_INFO("Sending first goal");                                 // ...and send it

  ac.sendGoal(goal);

  // If the waypoint remains the same, don't update the goal :)
  while(ros::ok()) {
    if (euclidianDifference(waypoint.pose.position.x, waypoint.pose.position.y, temp.pose.position.x, temp.pose.position.y) > waypointsMinDistance) {
      goal = setGoal(waypoint.pose.position.x, waypoint.pose.position.y);
      ROS_INFO("Sending goal");
      ac.sendGoal(goal);

      temp = waypoint;    // Get new waypoint from the topic

      //ac.waitForResult();
      //if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
      //  ROS_INFO("Hooray, the base reached the goal!");
      //else
      //  ROS_INFO("The base failed to reach the goal...");
    }
  }

  return 0;
}