#ifndef __KITTI_LOADER__
#define __KITTI_LOADER__

#include <string>
#include <boost/filesystem/path.hpp>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include <opencv2/opencv.hpp>

struct KittiFrame{
    cv::Mat left_img;
    cv::Mat right_img;
    pcl::PointCloud<pcl::PointXYZI>::Ptr ptcloud;
};

class KittiLoader{
public:
    KittiLoader(const std::string &data_root, bool &if_success);

    size_t Size(){return size_;}

    KittiFrame operator[](size_t i) const;
    
    static pcl::PointCloud<pcl::PointXYZI>::Ptr LoadPtCloud(const std::string &path);

private:
    size_t GetFileNumInDir(const boost::filesystem::path &p);

    boost::filesystem::path root_;
    boost::filesystem::path lidar_path_;
    boost::filesystem::path left_img_path_;
    boost::filesystem::path right_img_path_;

    size_t size_ = 0;
};

#endif