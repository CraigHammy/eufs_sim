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
#include <map_converter_pkg/ConvertMap.h>
#include "tf2/LinearMath/Matrix3x3.h"
#include "geometry_msgs/Quaternion.h"
#include <geometry_msgs/Point.h>
#include <Eigen/Core>
#include <Eigen/Dense>
#include "map_converter.hpp"


void MapConverter::printData()
{
    std::vector<geometry_msgs::Point>::const_iterator cone;
    int cone_number = 1;
    for(cone =cones_.begin(); cone !=cones_.end(); ++cone, ++cone_number)
    {
        ROS_INFO("Cone no.%d location => x: %f, y: %f", cone_number, cone->x, cone->y);
    }
}

void MapConverter::generateMapBoundaries()
{
    float highest = -1000;

    std::vector<geometry_msgs::Point>::const_iterator cone;
    for(cone =cones_.begin(); cone !=cones_.end(); ++cone)
    {
        if (abs(cone->x) > highest)
            highest = abs(cone->y);
        else if (abs(cone->x) > highest)
            highest = abs(cone->y); 
    }

    int length = int(highest) + (10 -(int(highest) % 10));
    int cell_length = CELL_NUM(resolution_, length) * 2;
    width_ = cell_length;
    height_ = cell_length;
}

void MapConverter::initialise()
{
    occupancy_grid_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>("track_map", 1, true);
    map_metadata_pub_ = nh_.advertise<nav_msgs::MapMetaData>("track_map_metadata", 1, true);
    convert_map_service_ = nh_.advertiseService("convert_map", &MapConverter::mapCallback, this);

    occ_thresh_ = 25;
    cone_radius_ = 0.114;
    resolution_ = 0.1;
    got_map_ = false;
    interpolation_thresh_ = 1.25;
}

bool MapConverter::mapCallback(map_converter_pkg::ConvertMap::Request& req, map_converter_pkg::ConvertMap::Response& res)
{
    ROS_INFO("Converting map");
    cones_ = req.landmarks;

    initialise(); 
    generateMapBoundaries();
    createMap();
    
    if(got_map_)
    {
        res = map_;
        return true;
        ROS_INFO("Service finished.");
    }
}

std::vector<geometry_msgs::Point> MapConverter::readCSV()
{
    //create input filestream
    std::ifstream file(file_name_.c_str());

    //make sure the file is open
    if (!file.is_open()) throw std::runtime_error("Could not open file");

    //helpers
    std::vector<geometry_msgs::Point> locations;
    std::string line;

    //skip first line
    std::getline(file, line);

    //store all x and y coordinates in following lines
    while (std::getline(file, line))
    {
        std::istringstream ss(line);
        std::string token;
        std::vector<std::string> row;

        while (std::getline(ss, token, delimiter_))
        {
            //process each token
            row.push_back(token);
        }

        //check that they are cones are not other objects from the csv file 
        if ( (row[0].compare("yellow"))==0)
        {
            geometry_msgs::Point cone;
            cone.x = std::atof(row[1].c_str());
            cone.y = std::atof(row[2].c_str());
           cones_.push_back(cone);
           yellow_cones_.push_back(cone);
        }
        else if ((row[0].compare("blue"))==0)
        {
            geometry_msgs::Point cone;
            cone.x = std::atof(row[1].c_str());
            cone.y = std::atof(row[2].c_str());
           cones_.push_back(cone);
           blue_cones_.push_back(cone);
        } 
        else if ((row[0].compare("big_orange"))==0)
        {
            geometry_msgs::Point cone;
            cone.x = std::atof(row[1].c_str());
            cone.y = std::atof(row[2].c_str());
           cones_.push_back(cone);
        }
    }
}

void MapConverter::createMap()
{
    //initialise map info parameters 
    map_.map.info.resolution = resolution_;
    //real world coordinates of the 0,0 cell origin (bottom left corner of map)
    map_.map.info.origin.position.x = POSE_VAL(resolution_, -width_ / 2);
    map_.map.info.origin.position.y = POSE_VAL(resolution_, -height_ / 2);
    map_.map.info.origin.position.z = 0.0;
    map_.map.info.origin.orientation.x = 0.0;
    map_.map.info.origin.orientation.y = 0.0;
    map_.map.info.origin.orientation.z = 0.0;
    map_.map.info.origin.orientation.w = 1.0;
    map_.map.info.height = height_;
    map_.map.info.width = width_;
    map_.map.data.resize(map_.map.info.width * map_.map.info.height);

    //sort the unknown, free and occupied space in the map
    int occupied = 0;
    int free = 0;
    int unknown = 0;
    for (int x = 0; x != width_; ++x)
    {
        for (int y = 0; y != height_; ++y)
        {
            float occ = getCell(x, y);
            if (occ < 0)
                map_.map.data[MAP_IDX(map_.map.info.width, x, y)] = -1; //unknown 
            else if (occ > occ_thresh_)
                map_.map.data[MAP_IDX(map_.map.info.width, x, y)] = 100; //occupied
            else
                map_.map.data[MAP_IDX(map_.map.info.width, x, y)] = 0; //free       
        }
    }
    got_map_ = true;

    //set header information
    map_.map.header.stamp = ros::Time::now();
    map_.map.header.frame_id = "map";
    ROS_INFO("Occupancy grid map created from list of x and y geometry_msgs::Points");

    occupancy_grid_pub_.publish(map_.map);
    map_metadata_pub_.publish(map_.map.info);
}

float MapConverter::getCell(int x, int y)
{
    float pose_x = resolution_ * (x - width_ / 2);
    float pose_y = resolution_ * (y - height_ / 2);

    float occ;
    float min_x, max_x, min_y, max_y;

    std::vector<geometry_msgs::Point>::const_iterator cone;
    for(cone =cones_.begin(); cone !=cones_.end(); ++cone)
        {
            min_x = cone->x - cone_radius_;
            max_x = cone->x + cone_radius_;
            min_y = cone->y - cone_radius_;
            max_y = cone->y + cone_radius_;

            if ((pose_x > min_x && pose_x < max_x) && (pose_y > min_y && pose_y < max_y))
            {
                occ = 100;
                break;
            } 
            else 
                occ = 0;
        }
}

float MapConverter::lagrangeInterpolation(float x_input, std::vector<float> xs, std::vector<float> ys)
{
    const int N = xs.size();
    float p, y_out;
    for (int i = 0; i != N; ++i)
    {
        p = 1;
        for (int j = 0; j != N; ++j)
        {
            if (i != j)
                p *= (x_input - xs[j]) / (xs[i] - xs[j]);
        }
        y_out += p * ys[i];
    }
    return y_out;
}

void MapConverter::saveMap(const std::string& map_name)
{
    std::string map_pgm_file = folder_name_ + map_name + ".pgm";
    ROS_INFO("Writing grid map occupancy data to %s", map_pgm_file.c_str());

    FILE* pgm_file;
    pgm_file= fopen(map_pgm_file.c_str(), "w");

    if (pgm_file == NULL)
    {
        ROS_ERROR("Couldn't save map file to %s", map_pgm_file.c_str());
        return;
    }

    fprintf(pgm_file, "P5\n# https://github.com/CraigHammy/eufs_sim %.3f m/pix\n%d %d\n255\n",
            map_.map.info.resolution, map_.map.info.width, map_.map.info.height);

    for(int y = 0; y < map_.map.info.height; y++) 
    {
        for(int x = 0; x < map_.map.info.width; x++) 
        {
            int i = x + (map_.map.info.height - y - 1) * map_.map.info.width;
            if (map_.map.data[i] >= 0 && map_.map.data[i] <= 25) 
            { 
                fputc(254, pgm_file); // [0,free)
            } else if (map_.map.data[i] >= 65) { // (occ,255]
                fputc(000, pgm_file);
            } else { //occ [0.25,0.65] unknown 
                fputc(205, pgm_file);
            }
        }
    }
    fclose(pgm_file);

    std::string map_metadata_file = folder_name_ + map_name + ".yaml";
    ROS_INFO("Writing map occupancy data to %s", map_metadata_file.c_str());

    FILE* yaml;
    yaml = fopen(map_metadata_file.c_str(), "w");

    geometry_msgs::Quaternion orientation = map_.map.info.origin.orientation;
    tf2::Quaternion quat(orientation.x, orientation.y, orientation.z, orientation.w);
    tf2::Matrix3x3 mat(quat);
    double yaw, pitch, roll;
    mat.getEulerYPR(yaw, pitch, roll);

    fprintf(yaml, "image: %s\nresolution: %f\norigin: [%f, %f, %f]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n\n",
            map_pgm_file.c_str(), map_.map.info.resolution, map_.map.info.origin.position.x, map_.map.info.origin.position.y, yaw);
    fclose(yaml);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "a_node");
    ros::NodeHandle nh("~");

    //std::string file_path;
    //nh.getParam("csv_map_path", file_path);
    //std::cout << file_path << std::endl;

    std::string output_folder;
    nh.getParam("converted_map_folder", output_folder);

    MapConverter mc(&nh, output_folder);

    ros::spin();

    return 0;
}