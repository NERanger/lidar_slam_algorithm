#include <iostream>
#include <string>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/range_image/range_image.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/range_image_visualizer.h>

#include "KittiLoader.hpp"
#include "Timer.hpp"

using std::cout;
using std::endl;
using std::cerr;
using std::string;

// Ref: https://pcl.readthedocs.io/projects/tutorials/en/latest/range_image_creation.html#range-image-creation

int main(int argc, char const *argv[]){
    if(argc != 2){
        cout << "Usage: ./gen_range_img <path-to-pointcloud>" << endl;
        return EXIT_FAILURE;
    }

    string path(argv[1]);

    Timer timer;
    int64_t span = 0;

    // Load pointcloud
    timer.Start();
    pcl::PointCloud<pcl::PointXYZI>::Ptr 
        ptcloud = KittiLoader::LoadPtCloud(path);
    span = timer.Stop();
    cout << "Frame load time (ms): " << span << endl;
    cout << "Total point number: " << ptcloud->size() << endl;

    float angular_resolution_x = (float) (0.08f * (M_PI/180.0f));  //   0.08 degree in radians
    float angular_resolution_y = (float) (0.4f * (M_PI/180.0f));  //   0.4 degree in radians
    float max_angle_width     = (float) (360.0f * (M_PI/180.0f));  // 360.0 degree in radians
    float max_angle_height    = (float) (180.0f * (M_PI/180.0f));  // 180.0 degree in radians
    float noise_level = 0.00;
    float min_range = 5.0f;
    int border_size = 1;

    Eigen::Affine3f sensor_pose = (Eigen::Affine3f)Eigen::Translation3f(0.0f, 0.0f, 0.0f);
    pcl::RangeImage::CoordinateFrame coordinate_frame = pcl::RangeImage::LASER_FRAME;

    pcl::RangeImage range_img;

    timer.Start();
    range_img.createFromPointCloud(*ptcloud, angular_resolution_x, angular_resolution_y, 
                                    max_angle_width, max_angle_height, sensor_pose, 
                                    coordinate_frame, noise_level, min_range, border_size);
    span = timer.Stop();
    cout << "Range img creation time (ms): " << span << endl;
    cout << range_img << endl;

    // Visualization
    pcl::visualization::PCLVisualizer::Ptr cloud_viewer
        (new pcl::visualization::PCLVisualizer("Viewer"));
    cloud_viewer->initCameraParameters();
    cloud_viewer->setBackgroundColor(0.0, 0.0, 0.0);
    pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZI> color(ptcloud, "intensity");
    cloud_viewer->addPointCloud<pcl::PointXYZI>(ptcloud, color, "origin_cloud");

    pcl::visualization::RangeImageVisualizer range_img_viewer("range_image");
    range_img_viewer.showRangeImage(range_img);

    while(!cloud_viewer->wasStopped()){
        cloud_viewer->spinOnce();
        range_img_viewer.spinOnce();
        pcl_sleep(0.01);
    }
    
    return EXIT_SUCCESS;
}