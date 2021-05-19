/**
 * File: KittiLoader.cc
 * Author: Jing Yonglin
 * Description: Class implementation for KITTI loader
 */

#include <memory>
#include <fstream>
#include <vector>

#include <boost/format.hpp>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include "KittiLoader.hpp"

using std::ifstream;
using std::string;
using std::vector;

using boost::format;

using pcl::PointCloud;
using pcl::PointXYZI;

PointCloud<PointXYZI>::Ptr KittiLoader::NextFrame(){
    format lidar_fmt("%s/velodyne/%06d.bin");
    string lidar_data_path = (lidar_fmt % path_ % current_idx_).str();

    current_idx_ += 1;

    return LoadSingleFrame(lidar_data_path);
}

PointCloud<PointXYZI>::Ptr KittiLoader::LoadSingleFrame(const string &path){
    ifstream lidar_data_file(path, ifstream::in | ifstream::binary);

    lidar_data_file.seekg(0, std::ios::end);
    const size_t num_elements = lidar_data_file.tellg() / sizeof(float);
    lidar_data_file.seekg(0, std::ios::beg);

    vector<float> lidar_data(num_elements);
    lidar_data_file.read(reinterpret_cast<char*>(&lidar_data.front()), 
                         num_elements * sizeof(float));

    PointCloud<PointXYZI>::Ptr lidar_frame_ptr(new PointCloud<PointXYZI>);
    for(size_t i = 0; i < lidar_data.size(); i += 4){
        PointXYZI p;
        p.x = lidar_data[i];
        p.y = lidar_data[i + 1];
        p.z = lidar_data[i + 2];
        p.intensity = lidar_data[i + 3];
        lidar_frame_ptr->push_back(p);
    }

    return lidar_frame_ptr;
}