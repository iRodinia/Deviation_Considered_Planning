#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <algorithm>
#include <iostream>
#include <vector>
#include <string>

#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

#include "global_map_generator/maps.hpp"

void optimizeMap(mocka::Maps::BasicInfo& in)
{
    std::vector<int>* temp = new std::vector<int>;

    pcl::KdTreeFLANN<pcl::PointXYZ>     kdtree;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

    cloud->width  = in.cloud->width;
    cloud->height = in.cloud->height;
    cloud->points.resize(cloud->width * cloud->height);

    for (uint32_t i = 0; i < cloud->width; i++)
    {
        cloud->points[i].x = in.cloud->points[i].x;
        cloud->points[i].y = in.cloud->points[i].y;
        cloud->points[i].z = in.cloud->points[i].z;
    }

    kdtree.setInputCloud(cloud);
    double radius = 1.75 / in.scale; // 1.75 is the rounded up value of sqrt(3)

    for (uint32_t i = 0; i < cloud->width; i++)
    {
        std::vector<int>   pointIdxRadiusSearch;
        std::vector<float> pointRadiusSquaredDistance;
        if (kdtree.radiusSearch(cloud->points[i], radius, pointIdxRadiusSearch,
                                pointRadiusSquaredDistance) >= 27)
        {
        temp->push_back(i);
        }
    }
    for (int i = temp->size() - 1; i >= 0; i--)
    {
        in.cloud->points.erase(in.cloud->points.begin() +
                            temp->at(i)); // erasing the enclosed points
    }
    in.cloud->width -= temp->size();

    pcl::toROSMsg(*in.cloud, *in.output);
    in.output->header.frame_id = "world";
    ROS_INFO("finish: number of points after optimization %d", in.cloud->width);
    delete temp;
    return;
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "global_map");
    ros::NodeHandle nh;

    int seed;
    int sizeX;
    int sizeY;
    int sizeZ;
    double resolution;
    double update_freq;
    int type;
    std::string _map_pub_topic = std::string("/global_map");

    nh.param("/global_map/seed", seed, 94);
    nh.param("/global_map/update_freq", update_freq, 1.0);
    nh.param("/global_map/resolution", resolution, 0.05);
    nh.param("/global_map/x_length", sizeX, 100);
    nh.param("/global_map/y_length", sizeY, 100);
    nh.param("/global_map/z_length", sizeZ, 10);
    nh.param("/global_map/map_type", type, 1);  // 1: perlin3D, 2: randomMap, 3: maze2D, 4: maze3D

    std::string map_type;
    if(type == 1){
        map_type = "perlin3D";
    }
    else if(type == 2){
        map_type = "randomMap";
    }
    else if(type == 3){
        map_type = "maze2D";
    }
    else{
        map_type = "maze3D";
    }
    ROS_INFO("Generate global map with parameters: seed (%d), resolution (%f), voxel_dim (%d,%d,%d), map_type (%s)", 
                seed, resolution, sizeX, sizeY, sizeZ, map_type.c_str());

    ros::Publisher pcl_pub = nh.advertise<sensor_msgs::PointCloud2>(_map_pub_topic, 1);
    pcl::PointCloud<pcl::PointXYZ> cloud;
    sensor_msgs::PointCloud2 output;
    double scale = 1 / resolution;

    mocka::Maps::BasicInfo info;
    info.nh_private = &nh;
    info.sizeX      = sizeX;
    info.sizeY      = sizeY;
    info.sizeZ      = sizeZ;
    info.seed       = seed;
    info.scale      = scale;
    info.output     = &output;
    info.cloud      = &cloud;

    mocka::Maps map;
    map.setInfo(info);
    map.generate(type);

    //! @note publish loop
    ros::Rate loop_rate(update_freq);
    while (ros::ok())
    {
        pcl_pub.publish(output);
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
