#include <iostream>
#include <string>
#include <fstream>
#include <vector>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/passthrough.h>
#include <pcl/visualization/cloud_viewer.h>

#include "Timer.hpp"
#include "KittiLoader.hpp"

using std::cout;
using std::cerr;
using std::endl;
using std::string;

int main(int argc, char const *argv[]){
    if(argc != 2){
        cout << "Usage: ./pass_through_filter <path-to-pointcloud>" << endl;
        return EXIT_FAILURE;
    }

    string path(argv[1]);

    Timer timer;
    int64_t span = 0;

    // Load pointcloud
    timer.Start();
    pcl::PointCloud<pcl::PointXYZI>::Ptr 
        ptcloud = KittiLoader::LoadSingleFrame(path);
    span = timer.Stop();
    cout << "Frame load time (ms): " << span << endl;
    cout << "Total point number: " << ptcloud->size() << endl;

    pcl::PassThrough<pcl::PointXYZI> pt_filter;
    pt_filter.setInputCloud(ptcloud);
    pt_filter.setFilterFieldName("x");  // Filter x axis data only
    pt_filter.setFilterLimits(0.0, 10.0);
    // Uncomment the following line to try out
    // pt_filter.setFilterLimitsNegative(true);

    // Filtered_cloud
    pcl::PointCloud<pcl::PointXYZI>::Ptr 
        filtered_cloud(new pcl::PointCloud<pcl::PointXYZI>);

    timer.Start();
    // Perform filtering
    pt_filter.filter(*filtered_cloud);
    span = timer.Stop();
    cout << "Filtering time (ms): " << span << endl;

    pcl::visualization::PCLVisualizer::Ptr viewer
        (new pcl::visualization::PCLVisualizer("Viewer"));
    viewer->initCameraParameters();

    // ID for viewport 1
    int vp_1 = 0;
    viewer->createViewPort(0.0, 0.0, 0.5, 1.0, vp_1);
    viewer->setBackgroundColor(0.0, 0.0, 0.0, vp_1);
    viewer->addText("Original pointcloud", 10, 10, "vp1_text", vp_1);
    pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZI> vp1_color(ptcloud, "x");
    viewer->addPointCloud<pcl::PointXYZI>(ptcloud, vp1_color, "origin_cloud", vp_1);

    // ID for viewport 2
    int vp_2 = 1;
    viewer->createViewPort(0.5, 0.0, 1.0, 1.0, vp_2);
    viewer->setBackgroundColor(0.5, 0.5, 0.5, vp_2);
    viewer->addText("Filtered pointcloud", 10, 10, "vp2_text", vp_2);
    pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZI> vp2_color(filtered_cloud, "x");
    viewer->addPointCloud<pcl::PointXYZI>(filtered_cloud, vp2_color, "filtered_cloud", vp_2);

    viewer->addCoordinateSystem(1.0);

    viewer->spin();

    return EXIT_SUCCESS;
}
