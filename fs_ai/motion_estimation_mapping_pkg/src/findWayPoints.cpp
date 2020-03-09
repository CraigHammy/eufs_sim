#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <perception_pkg/Cone.h>
#include <geometry_msgs/Point.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Odometry.h>
#include <Eigen/Core>

/**
 * @class FindWayPoints
 * @brief ROS functions to take PoseStamped cone detections as inputs and returns middle points between each pair of consecutive cones. 
 */
class FindWayPoints
{
public:
    /**
     * @brief Constructor for FindWayPoints class 
     */
    FindWayPoints(ros::NodeHandle* nh) : nh_(*nh), initialised_(false), goal_publisher_ready_(false),
        odom_subscriber_active_(false), cone_subscriber_active_(false), count_(0) {}

    /**
     * @brief Initialisers subscribers, publishers, helper variables and parameters 
     */ 
    void initialise();

    /**
     * @brief Finds the middle point between a pair of yellow and blue cones
     * @param cone_left PoseStamped of left cone of pair
     * @param cone_right PoseStamped of right cone of pair 
     */
    void generateWayPoint(const geometry_msgs::Point::ConstPtr& cone_left, const geometry_msgs::Point::ConstPtr& cone_right);

    /**
     * @brief Matches cones into pairs from cones detected by computer vision pipeline 
     * @return Tuple of correctly coupled cone pair
     */
    std::pair<geometry_msgs::PoseStamped, geometry_msgs::PoseStamped> optimiseConePairs();

    /**
     * @brief Wraps angles from ROS format [-180, +180] to [0, +360]
     * @param angle Angle in radians
     * @return Wrapped angle in radians  
     */
    double angleWrap(double angle)
    {
        if (angle < 0) angle += 2 * M_PI;
        return angle;
    }
    
private:
    /**
     * @brief Waits for publiser to be ready to publish messages
     * @param publisher Topic publisher 
     */
    void waitForPublisher();

    /**
     * @brief Waits for subscribers to start receiving messages 
     */
    void waitForSubscribers();

    /**
     * @brief Callback for receiving odometry data
     * @param msg An Odometry message
     */
    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);

    /**
     * @brief Callback for receiving detected Cone data
     * @param msg A Cone message
     */
    void coneCallback(const perception_pkg::Cone::ConstPtr& msg);

    /**
     * @brief Transforms from a LiDAR or camera frame to map frame
     * @param msg A Cone message
     * @return A StampedTransform in map frame 
     */
    tf::StampedTransform getTransform(const perception_pkg::Cone::ConstPtr& msg);

    /**
     * @brief Calculates distance between the point and the base_link of the robot 
     * @param point A Point message
     * @return The distance from the point to the base_link of the robot as a double
     */
    double calculateDistanceFromBaseLink(const geometry_msgs::Point::ConstPtr& point);

    /**
     * @brief Calculates distance between two points 
     * @param point1 A Point message
     * @param point2 A Point message
     * @return The distance between the two cones as a double
     */
    double calculateDistanceBetweenCones(const geometry_msgs::Point::ConstPtr& point1, const geometry_msgs::Point::ConstPtr& point2);

    /**
     * @brief Determines if new detected cone point has already been detected
     * @param A Point message
     * @return True if point is a new cone detection, false otherwise
     */
    bool isNewDetection(const geometry_msgs::Point::ConstPtr& point);

    /**
     * @brief Compares two cone points based on distance from the base_link robot frame 
     * @param point1 A Cone message
     * @param point2 A Cone message 
     * @return True if the distance from base_link to point1 is smaller than the distance from base_link to poin2, false otherwise 
     */
    bool compareDistances(const geometry_msgs::Point::ConstPtr& point1, const geometry_msgs::Point::ConstPtr& point2);

    /**
     * @brief Compares two cone points based on distance from the base_link robot frame 
     * @param point1 A Cone message (not pointer)
     * @param point2 A Cone message  (not pointer)
     * @return True if the distance from base_link to point1 is smaller than the distance from base_link to poin2, false otherwise 
     */
    bool compareDistancesForSort(const geometry_msgs::Point& point1, const geometry_msgs::Point& point2);

    double acc_min_consec_dist, acc_max_consec_dist;
    double acc_min_lane_width, acc_max_lane_width;
    double sprint_min_consec_inner_dist, sprint_max_consec_inner_dist;
    double sprint_min_consec_outer_dist, sprint_max_consec_outer_dist;
    double sprint_min_lane_width, sprint_max_lane_width;
    std::string odom_topic, goal_topic, cone_topic;
    std::string distance_method;
    double detection_thresh, wait_time;

    boost::shared_ptr<geometry_msgs::Point> cone_ptr_;
    std::map<geometry_msgs::Point, std::string> unique_cones_;

    /*std::map<int, std::string> colour_map = {
    { 0, "yellow" },
    { 1, "blue"},
    { 2, "orange" },
    {3, "uknown"}
    };*/

    ros::NodeHandle nh_;
    ros::Subscriber odom_sub_, cone_sub_;
    ros::Publisher goal_pub_;
    double x_, y_, yaw_;
    bool goal_publisher_ready_, odom_subscriber_active_, cone_subscriber_active_;
    bool initialised_;
    int count_;
};

/**
 * @brief Initialisers subscribers, publishers, helper variables and parameters 
 */ 
void FindWayPoints::initialise()
{
    if(!initialised_) {
        nh_.param("odom", odom_topic);
        nh_.param("cone", cone_topic);
        odom_sub_ = nh_.subscribe<nav_msgs::Odometry>(odom_topic, 1, boost::bind(&FindWayPoints::odomCallback, this, _1));
        cone_sub_ = nh_.subscribe<perception_pkg::Cone>(cone_topic, 1, boost::bind(&FindWayPoints::coneCallback, this, _1));
        waitForSubscribers();

        nh_.param("goal", goal_topic);
        goal_pub_ = nh_.advertise<geometry_msgs::Point>(goal_topic, 1);
        waitForPublisher();

        nh_.param("acceleration_min_consecutive_dist", acc_min_consec_dist);
        nh_.param("acceleration_max_consecutive_dist", acc_max_consec_dist);
        nh_.param("acceleration_min_lane_width", acc_min_lane_width);
        nh_.param("acceleration_max_lane_width", acc_max_lane_width);

        nh_.param("sprint_min_consecutive_inner_dist", sprint_min_consec_inner_dist);
        nh_.param("sprint_max_consecutive_inner_dist", sprint_max_consec_inner_dist);
        nh_.param("sprint_min_consecutive_outer_dist", sprint_min_consec_outer_dist);
        nh_.param("sprint_max_consecutive_outer_dist", sprint_max_consec_outer_dist);
        nh_.param("sprint_min_lane_width", sprint_min_lane_width);
        nh_.param("sprint_max_lane_width", sprint_max_lane_width);

        nh_.param("distance_method", distance_method);
        nh_.param("detection_thresh", detection_thresh);
        nh_.param("wait_time", wait_time);

        initialised_ = true;
    } 
    else{
        ROS_WARN("FindWayPoints object has already been initialised, doing nothing.");
    }
}

/**
 * @brief Waits for publiser to be ready to publish messages 
 */
void FindWayPoints::waitForPublisher()
{
    ros::Rate rate(20);
    while (ros::ok()) {
        int num_connections = goal_pub_.getNumSubscribers();
        if (num_connections > 0) 
        {
            goal_publisher_ready_ = true;
            ROS_WARN("The publisher is ready.");
            break;
        }
    }
}

/**
 * @brief Waits for subscriber to start receiving messages
 */
void FindWayPoints::waitForSubscribers()
{
    while(!odom_subscriber_active_ || !cone_subscriber_active_) 
    ;
    ROS_WARN("The cone detection pipeline has started publishing cone locations.");
}

/**
 * @brief Callback for receiving odometry data
 * @param msg An Odometry message
 */
void FindWayPoints::odomCallback(const nav_msgs::Odometry::ConstPtr& msg) 
{
        x_, y_ = msg->pose.pose.position.x, msg->pose.pose.position.y;
        tf::Quaternion q(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
            msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
        tf::Matrix3x3 m(q);
        double roll, pitch;
        m.getRPY(roll, pitch, yaw_);
        double yaw_degrees = angleWrap(yaw_) * 180.0 / M_PI;
        ROS_DEBUG_NAMED("find_waypoints", "In the odometry callback with position values: (%.2f, %2f)" 
            " and orientation value in degrees: %.2f", x_, y_, yaw_degrees);
        if (count_ == 0)
            odom_subscriber_active_ = true;
}

/**
 * @brief Callback for receiving detected Cone data
 * @param msg A Cone message
 */
void FindWayPoints::coneCallback(const perception_pkg::Cone::ConstPtr& msg)
{
    //find the transform between the map and the cone links
    tf::StampedTransform transform(getTransform(msg));

    //create pointer to make program execution quicker when inserting it as parameter in function calls
    //reset makes a shared_ptr take ownership of a new object pointer, the object is was linked to before is deleted through destructor
    cone_ptr_.reset(new geometry_msgs::Point);
    cone_ptr_->x = transform.getOrigin().x(), transform.getOrigin().y(), transform.getOrigin().z();

    //check if detection is new and declare subscriber active when the first cone is detected
    if (isNewDetection(cone_ptr_)) 
    {
        if (count_ == 0)
            cone_subscriber_active_ = true;
        unique_cones_.insert(std::pair<geometry_msgs::Point, std::string>(*cone_ptr_, msg->colour));
    }
}

/**
 * @brief Transforms from a LiDAR or camera frame to map frame
 * @param msg A Cone message
 * @return A StampedTransform in map frame 
 */
tf::StampedTransform FindWayPoints::getTransform(const perception_pkg::Cone::ConstPtr& msg)
{
    //initialise TransformListener and StampedTransform
    static tf::TransformListener listener;
    tf::StampedTransform transform;

    //create expected link  
    std::string link_name = "/cone" + '_' + msg->header.seq + '_' + msg->colour;
    
    //block until transform becomes available or until timeout has been reached 
    ros::Time now = ros::Time::now();
    try{
        listener.waitForTransform("/map", link_name, now, ros::Duration(0.5));
        //WHAT HAPPENS IF THE TRANSFORM IS NOT BROADCASTED IN THE TIME WE WAIT ?? --> MAKE A FIX 
        listener.lookupTransform("/map", link_name, now, transform);
    }
    catch (tf::TransformException e){
        ROS_ERROR("%s", e.what());
        ros::Duration(1.0).sleep();
    }
    return transform;
}

/**
 * @brief Calculates distance between the point and the base_link of the robot 
 * @param point A Point message
 * @return The distance from the point to the base_link of the robot as a double
 */
double FindWayPoints::calculateDistanceFromBaseLink(const geometry_msgs::Point::ConstPtr& point)
{
    double distance;
    if (distance_method.compare("euclidean")) {
        distance = std::sqrt(std::pow(point->x - x_, 2) + std::pow(point->y - y_, 2));
    }else if (distance_method.compare("manhattan")) {
        distance = std::abs(point->x - x_) + std::abs(point->y - y_);
    }
    return distance;
}

/**
 * @brief Calculates distance between two points 
 * @param point1 A Point message
 * @param point2 A Point message
 * @return The distance between the two cones as a double
 */
double FindWayPoints::calculateDistanceBetweenCones(const geometry_msgs::Point::ConstPtr& point1, const geometry_msgs::Point::ConstPtr& point2)
{
    double distance;
    if (distance_method.compare("euclidean")) {
        distance = std::sqrt(std::pow(point2->x - point1->x, 2) + std::pow(point2->y - point1->y, 2));
    }else if (distance_method.compare("manhattan")) {
        distance = std::abs(point2->x - point1->x) + std::abs(point2->y - point1->y);
    }
    return distance;
}

/**
 * @brief Determines if new detected cone point has already been detected
 * @param A Point message
 * @return True if point is a new cone detection, false otherwise
 */
bool FindWayPoints::isNewDetection(const geometry_msgs::Point::ConstPtr& point)
{
    typedef std::map<geometry_msgs::Point, std::string>::const_iterator iter;
    double distance;
    bool new_detection(false);

    for(iter i = unique_cones_.begin(); i != unique_cones_.end(); ++i)
    {
        distance = calculateDistanceFromBaseLink(point);
        if (distance > detection_thresh)
            new_detection = true;
        return new_detection;
    }
}

/**
 * @brief Compares two cone points based on distance from the base_link robot frame 
 * @param point1 A Cone message
 * @param point2 A Cone message 
 * @return True if the distance from base_link to point1 is smaller than the distance from base_link to poin2, false otherwise 
 */
bool FindWayPoints::compareDistances(const geometry_msgs::Point::ConstPtr& point1, const geometry_msgs::Point::ConstPtr& point2)
{
    double distance1, distance2;
    distance1 = calculateDistanceFromBaseLink(point1);
    distance2 = calculateDistanceFromBaseLink(point2);
    return distance1 < distance2;
}

/**
 * @brief Compares two cone points based on distance from the base_link robot frame 
 * @param point1 A Cone message (not pointer)
 * @param point2 A Cone message  (not pointer)
 * @return True if the distance from base_link to point1 is smaller than the distance from base_link to poin2, false otherwise 
 */
bool FindWayPoints::compareDistancesForSort(const geometry_msgs::Point& point1, const geometry_msgs::Point& point2)
{
    double distance1, distance2;
    boost::shared_ptr<geometry_msgs::Point> point1_ptr(new geometry_msgs::Point(point1));
    boost::shared_ptr<geometry_msgs::Point> point2_ptr(new geometry_msgs::Point(point2));
    distance1 = calculateDistanceFromBaseLink(point1_ptr);
    distance2 = calculateDistanceFromBaseLink(point2_ptr);
    return distance1 < distance2;
}

/**
 * @brief Matches cones into pairs from cones detected by computer vision pipeline 
 * @return Tuple of correctly coupled cone pair
 */
std::pair<geometry_msgs::PoseStamped, geometry_msgs::PoseStamped> FindWayPoints::optimiseConePairs()
{
    bool lap_completed = false;
    std::vector<geometry_msgs::Point> blue_cones, yellow_cones, orange_cones, unknown_cones;

    std::vector<std::pair<geometry_msgs::Point, geometry_msgs::Point> > pairs;
    std::pair<geometry_msgs::Point, geometry_msgs::Point> possible_pair;
    typedef std::map<geometry_msgs::Point, std::string>::iterator iter;
    while (!lap_completed) 
    {
        //wait time for data to accumulate
        ros::Duration(wait_time).sleep();

        //update temporary lists and remove added elements from unique_cones_
        iter i = unique_cones_.begin();
        while(i != unique_cones_.end()) {

            if((i->second) == "blue")
            {
                blue_cones.push_back(i->first);
                unique_cones_.erase(i++);
            }
            else if((i->second) == "yellow")
            {
                yellow_cones.push_back(i->first);
                unique_cones_.erase(i++);
            }
            else if((i->second) == "orange")
            {
                orange_cones.push_back(i->first);
                unique_cones_.erase(i++);
            } 
            else if((i->second) == "unknown")
            {
                unknown_cones.push_back(i->first);
                unique_cones_.erase(i++);
            }
            else
            { 
                ++i;
            }
        }

        //if both orange cones are detected it means it is the last goal
        if (orange_cones.size() == 2)
        {
            possible_pair = std::make_pair(orange_cones[0], orange_cones[1]);
            pairs.push_back(possible_pair);
            lap_completed = true;
        }

        //conditions to go to next iteration of while loop 
        if (((blue_cones.empty() && yellow_cones.empty() && unknown_cones.size() < 2) || 
            (yellow_cones.empty() && unknown_cones.empty()) || (blue_cones.empty() && unknown_cones.empty())))
                continue;

        //order the vectors in increasing order based on their distance from the base_link 
        //this operation has a lot of complexity -> try using Eigen 
        std::sort(blue_cones.begin(), blue_cones.end(), boost::bind(&FindWayPoints::compareDistancesForSort, this, _1, _2));
        std::sort(yellow_cones.begin(), yellow_cones.end(), boost::bind(&FindWayPoints::compareDistancesForSort, this, _1, _2));
        std::sort(orange_cones.begin(), orange_cones.end(), boost::bind(&FindWayPoints::compareDistancesForSort, this, _1, _2));
        std::sort(unknown_cones.begin(), unknown_cones.end(), boost::bind(&FindWayPoints::compareDistancesForSort, this, _1, _2));

        //pair up a yellow and a blue and change one of the two with unknown if closer than any of the two 
        boost::shared_ptr<geometry_msgs::Point> unknown_ptr(new geometry_msgs::Point(unknown_cones[0]));
        boost::shared_ptr<geometry_msgs::Point> yellow_ptr(new geometry_msgs::Point(yellow_cones[0]));
        boost::shared_ptr<geometry_msgs::Point> blue_ptr(new geometry_msgs::Point(blue_cones[0]));
        double distance;
        if (compareDistances(unknown_ptr, blue_ptr)) {
            distance = calculateDistanceBetweenCones(yellow_ptr, unknown_ptr);
            if ((distance > sprint_min_lane_width) && (distance < sprint_max_lane_width))
            {
                possible_pair = std::make_pair(yellow_cones[0], unknown_cones[0]);
                pairs.push_back(possible_pair);
                //remove the added ones
                yellow_cones.erase(yellow_cones.begin());
                unknown_cones.erase(unknown_cones.begin());
            }
        } 
        else if (compareDistances(unknown_ptr, yellow_ptr)) {
            distance = calculateDistanceBetweenCones(blue_ptr, unknown_ptr);
            if ((distance > sprint_min_lane_width) && (distance < sprint_max_lane_width))
            {
                possible_pair = std::make_pair(blue_cones[0], unknown_cones[0]);
                pairs.push_back(possible_pair);
                //remove the added ones
                blue_cones.erase(blue_cones.begin());
                unknown_cones.erase(unknown_cones.begin());
            }
        }
        else {
            distance = calculateDistanceBetweenCones(yellow_ptr, blue_ptr);
            if ((distance > sprint_min_lane_width) && (distance < sprint_max_lane_width))
            {
                possible_pair = std::make_pair(yellow_cones[0], blue_cones[0]);
                pairs.push_back(possible_pair);
                //remove the added ones
                blue_cones.erase(blue_cones.begin());
                yellow_cones.erase(yellow_cones.begin());
            }
        }
    }
}

/**
 * @brief Finds and publishes the middle point between a pair of yellow and blue cones
 * @param cone_left PoseStamped of left cone of pair
 * @param cone_right PoseStamped of right cone of pair
 */
void FindWayPoints::generateWayPoint(const geometry_msgs::Point::ConstPtr& cone_left, const geometry_msgs::Point::ConstPtr& cone_right)
{
    geometry_msgs::PoseStamped goal;
    //take average of x and y cone positions 
    double x_mid = std::abs((cone_right->x + cone_left->x) / 2);
    double y_mid = std::abs((cone_right->y + cone_left->y) / 2);
    //put random yaw value and make the tolerance for the goal position very high 
    geometry_msgs::Point position;
    position.x, position.y, position.z = x_mid, y_mid, 0.0;
    geometry_msgs::Quaternion orientation;
    orientation.x, orientation.y, orientation.z, orientation.w = 0.0, 0.0, 0.0, 1.0;
    goal.pose.position = position;
    goal.pose.orientation = orientation;
    goal_pub_.publish(goal);
}

int main(int argc, char** argv) 
{
    //create node and node handle
    ros::init(argc, argv, "generate_waypoints_node");
    ros::NodeHandle nodeHandle;
    tf::TransformListener transformListener;

    //create object from node handle and spin the node 
    FindWayPoints fw(&nodeHandle);
    fw.initialise();
    fw.optimiseConePairs();
    ros::spin();
    return 0;
}