/**
 * File: KittiLoader.hpp
 * Author: Jing Yonglin
 * Description: Class definition for KITTI loader
 */

#ifndef LIDAR_ALGORITHM_KITTI_LOADER
#define LIDAR_ALGORITHM_KITTI_LOADER

#include <string>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

class KittiLoader{
public:
    KittiLoader(const std::string &path) : path_(path){};

    pcl::PointCloud<pcl::PointXYZI>::Ptr NextFrame();

    static pcl::PointCloud<pcl::PointXYZI>::Ptr LoadSingleFrame(const std::string &path);

private:
    std::string path_;
    size_t current_idx_ = 0;
};

#endif