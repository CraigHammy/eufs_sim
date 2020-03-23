#ifndef MAP_CONVERTER_HEADER
#define MAP_CONVERTER_HEADER

#include <ros/ros.h>
#include <fstream>
#include <vector>
#include <utility>
#include <stdexcept>
#include <sstream>
#include <string>
#include <iterator>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/MapMetaData.h>
#include <std_srvs/Empty.h>
#include "tf2/LinearMath/Matrix3x3.h"
#include "geometry_msgs/Quaternion.h"
#include <Eigen/Core>
#include <geometry_msgs/Point.h>
#include <Eigen/Dense>

//compute linear index for given map coords
#define MAP_IDX(width, x, y) ((width) * (x) + (y))

//compute pose from map parameters resolution and cell number
#define POSE_VAL(res, cell) (res * cell)

//compute cell number from map resolution and pose value
#define CELL_NUM(res, pose) (pose / res)

class MapConverter
{
public:

    MapConverter(ros::NodeHandle* nh,  std::string name, std::string folder): nh_(*nh), folder_name_(folder), file_name_(name) {initialise();};

    MapConverter(ros::NodeHandle* nh, char delim, std::string name, std::string folder): folder_name_(folder), file_name_(name), delimiter_(delim), nh_(*nh)
    { readCSV(); initialise(); generateMapBoundaries(); };

    
    bool mapCallback(map_converter_pkg::ConvertMap::Request& req, map_converter_pkg::ConvertMap::Response& res);    

private:
    void initialise();
    float getCell(int x, int y);
    float lagrangeInterpolation(float x_input, std::vector<float> xs, std::vector<float> ys);
    std::vector<geometry_msgs::Point> readCSV();
    void printData();
    void generateMapBoundaries();
    void createMap();
    void saveMap(const std::string& map_name);

    std::string file_name_;
    std::string folder_name_;
    char delimiter_;
    std::vector<geometry_msgs::Point>cones_;
    std::vector<geometry_msgs::Point> blue_cones_, yellow_cones_; 
    bool got_map_;

    float resolution_;
    int width_;
    int height_;
    float occ_thresh_;
    float cone_radius_;
    float interpolation_thresh_;

    ros::NodeHandle nh_;
    map_converter_pkg::ConvertMap::Response map_;
    ros::Publisher occupancy_grid_pub_;
    ros::Publisher map_metadata_pub_;
    ros::ServiceServer convert_map_service_;
};

#endif