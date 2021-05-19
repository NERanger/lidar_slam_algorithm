#include <iostream>
#include <string>
#include <fstream>
#include <vector>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/features/normal_3d.h>
#include <pcl/visualization/pcl_visualizer.h>

#include "Timer.hpp"
#include "KittiLoader.hpp"

using std::cout;
using std::cerr;
using std::endl;
using std::string;
using std::ifstream;
using std::vector;

using pcl::PointCloud;
using pcl::PointXYZI;
using pcl::NormalEstimation;
using pcl::Normal;
using pcl::search::KdTree;

int main(int argc, char const *argv[]){
    if(argc != 2){
        cout << "Usage: single_cloud_feat_extraction <path-to-pointcloud>" << endl;
        return EXIT_FAILURE;
    }

    Timer timer;
    int64_t span = 0;

    timer.Start();
    string path(argv[1]);
    PointCloud<PointXYZI>::Ptr ptcloud = KittiLoader::LoadSingleFrame(path);
    span = timer.Stop();
    cout << "Frame load time (ms): " << span << endl;
    cout << "Total number of points in frame: " << ptcloud->size() << endl;

    timer.Start();
    NormalEstimation<PointXYZI, Normal> n;
    n.setInputCloud(ptcloud);

    KdTree<PointXYZI>::Ptr tree(new KdTree<PointXYZI>());
    n.setSearchMethod(tree);

    PointCloud<Normal>::Ptr cloud_normals(new PointCloud<Normal>);
    n.setRadiusSearch(0.20); // Estimation based on points within 10cm radius
    n.compute(*cloud_normals);
    span = timer.Stop();
    cout << "Normal Estimation time (ms): " << span << endl;
    cout << "Normal cloud size: " << cloud_normals->size() << endl;

    pcl::visualization::PCLVisualizer::Ptr
        viewer(new pcl::visualization::PCLVisualizer("Viewer"));
    viewer->setBackgroundColor(0.0, 0.0, 0.0);

    // Set original point color as green
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZI> 
        single_color(ptcloud, 0, 255, 0);
    
    viewer->addPointCloud<PointXYZI>(ptcloud, single_color, "original cloud");
    viewer->addPointCloudNormals<PointXYZI, Normal>(ptcloud, cloud_normals, 20, 0.5, "normals");
    viewer->addCoordinateSystem(1.0);
    viewer->initCameraParameters();

    viewer->spin();

    return EXIT_SUCCESS;
}