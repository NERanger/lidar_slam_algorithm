#include <iostream>
#include <string>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/visualization/pcl_visualizer.h>

#include "KittiLoader.hpp"
#include "Timer.hpp"

using std::cout;
using std::cerr;
using std::endl;
using std::string;

// Ref: https://pcl.readthedocs.io/projects/tutorials/en/latest/statistical_outlier.html#statistical-outlier-removal

int main(int argc, char const *argv[]){
    if(argc != 2){
        cout << "Usage: ./stat_outlier_removal_filter <path-to-pointcloud>" << endl;
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

    // Create filter obj
    pcl::StatisticalOutlierRemoval<pcl::PointXYZI> sor;
    sor.setInputCloud(ptcloud);
    sor.setMeanK(50);  // Use nearest 50 points for mean calculation
    sor.setStddevMulThresh(1.0);

    // Filtered_cloud
    pcl::PointCloud<pcl::PointXYZI>::Ptr 
        filtered_cloud(new pcl::PointCloud<pcl::PointXYZI>);
    
    // Perform filtering
    timer.Start();
    sor.filter(*filtered_cloud);
    span = timer.Stop();
    cout << "Statistical Outlier Removal filtering time (ms): " << span << endl;
    cout << "Filtered cloud size: " << filtered_cloud->size() << endl;

    // Visualization
    pcl::visualization::PCLVisualizer::Ptr viewer
        (new pcl::visualization::PCLVisualizer("Viewer"));
    viewer->initCameraParameters();

    // ID for viewport 1
    int vp_1 = 0;
    viewer->createViewPort(0.0, 0.0, 0.5, 1.0, vp_1);
    viewer->setBackgroundColor(0.0, 0.0, 0.0, vp_1);
    viewer->addText("Original pointcloud", 10, 10, "vp1_text", vp_1);
    pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZI> vp1_color(ptcloud, "intensity");
    viewer->addPointCloud<pcl::PointXYZI>(ptcloud, vp1_color, "origin_cloud", vp_1);

    // ID for viewport 2
    int vp_2 = 1;
    viewer->createViewPort(0.5, 0.0, 1.0, 1.0, vp_2);
    viewer->setBackgroundColor(0.3, 0.3, 0.3, vp_2);
    viewer->addText("Filtered pointcloud", 10, 10, "vp2_text", vp_2);
    pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZI> vp2_color(filtered_cloud, "intensity");
    viewer->addPointCloud<pcl::PointXYZI>(filtered_cloud, vp2_color, "filtered_cloud", vp_2);

    viewer->addCoordinateSystem(1.0);

    viewer->spin();
    
    return EXIT_SUCCESS;
}
