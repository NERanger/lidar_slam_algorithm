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

    // Lower original pointcloud 10 for visualization
    for(size_t i = 0; i < ptcloud->size(); ++i){
        ptcloud->points[i].z -= 10.0;
    }

    pcl::visualization::CloudViewer viewer("Viewer");

    viewer.showCloud(ptcloud, "original cloud");
    viewer.showCloud(filtered_cloud, "filtered cloud");

    while(!viewer.wasStopped()){}

    return EXIT_SUCCESS;
}
