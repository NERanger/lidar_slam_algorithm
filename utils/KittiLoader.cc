#include <iostream>
#include <string>
#include <vector>
#include <fstream>

#include <boost/filesystem.hpp>
#include <boost/format.hpp>

#include <opencv2/opencv.hpp>

#include "KittiLoader.hpp"

using std::cout;
using std::cerr;
using std::endl;
using std::string;
using std::ifstream;
using std::vector;

KittiLoader::KittiLoader(const string &data_root, bool &if_success) : root_(data_root){
    using boost::filesystem::exists;

    lidar_path_ = root_ / ("velodyne");
    left_img_path_ = root_ / ("image_0");
    right_img_path_ = root_ / ("image_1");

    cout << "[KittiLoader] Desired data path:" << endl
         << "-- LiDAR data: " << lidar_path_.string() << endl
         << "-- Left camera image: " << left_img_path_.string() << endl
         << "-- Right camera image: " << right_img_path_.string() << endl;

    if(!(exists(lidar_path_) && exists(left_img_path_) && exists(right_img_path_))){
        cerr << "[KittiLoader] Dataset not complete, check if the desired data path exists." << endl;
        if_success = false;
    }

    size_t lidar_data_num = GetFileNumInDir(lidar_path_);
    size_t left_img_num = GetFileNumInDir(left_img_path_);
    size_t right_img_num = GetFileNumInDir(right_img_path_);

    if (! (lidar_data_num == left_img_num && left_img_num == right_img_num)){
        cerr << "[KittiLoader] Dataset not complete, frame number of LiDAR, "
             << "left camera and right camera not equal" << endl; 
        if_success = false;
    }
    
    size_ = lidar_data_num;
    if_success = true;
}

KittiFrame KittiLoader::operator[](size_t i) const{
    using boost::format;
    format fmt_lidar("%s/%06d.bin");
    format fmt_img("%s/%06d.png");

    string ptcloud = (fmt_lidar % lidar_path_.string() % i).str();
    string left_img = (fmt_img % left_img_path_.string() % i).str();
    string right_img = (fmt_img % right_img_path_.string() % i).str();

    KittiFrame f;
    f.ptcloud = LoadPtCloud(ptcloud);
    f.left_img = cv::imread(left_img);
    f.right_img = cv::imread(right_img);

    return f;
}

// Reference: https://stackoverflow.com/questions/41304891/how-to-count-the-number-of-files-in-a-directory-using-standard
size_t KittiLoader::GetFileNumInDir(const boost::filesystem::path &p){
    using boost::filesystem::directory_iterator;
    return std::distance(directory_iterator(p), directory_iterator{});
}

pcl::PointCloud<pcl::PointXYZI>::Ptr KittiLoader::LoadPtCloud(const string &path){
    using pcl::PointCloud;
    using pcl::PointXYZI;

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

    lidar_frame_ptr->width = lidar_frame_ptr->size();
    lidar_frame_ptr->height = 1;

    return lidar_frame_ptr;
}