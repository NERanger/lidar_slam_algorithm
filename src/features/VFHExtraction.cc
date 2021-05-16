#include <iostream>
#include <string>
#include <fstream>
#include <vector>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/vfh.h>

#include "Timer.hpp"
#include "KittiLoader.hpp"

using std::cout;
using std::cerr;
using std::endl;
using std::string;
using std::ifstream;
using std::vector;

// Reference: https://pcl.readthedocs.io/projects/tutorials/en/latest/vfh_estimation.html#vfh-estimation

int main(int argc, char const *argv[]){
    if(argc != 2){
        cout << "Usage: vfh_extraction <path-to-pointcloud>" << endl;
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

    // Normal estimation
    timer.Start();
    pcl::search::KdTree<pcl::PointXYZI>::Ptr 
        tree(new pcl::search::KdTree<pcl::PointXYZI>());

    pcl::NormalEstimation<pcl::PointXYZI, pcl::Normal> n;
    n.setInputCloud(ptcloud);
    n.setSearchMethod(tree);

    pcl::PointCloud<pcl::Normal>::Ptr 
        cloud_normals(new pcl::PointCloud<pcl::Normal>);
    n.setRadiusSearch(0.10); // Estimation based on points within 10cm radius
    span = timer.Stop();
    cout << "Normal Estimation preparation time (ms): " << span << endl;

    timer.Start();
    n.compute(*cloud_normals);
    span = timer.Stop();
    cout << "Normal Estimation time (ms): " << span << endl;

    // Create the VFH estimation class, and pass the input dataset+normals to it
    timer.Start();
    pcl::VFHEstimation<pcl::PointXYZI, pcl::Normal, pcl::VFHSignature308> vfh;
    vfh.setInputCloud(ptcloud);
    vfh.setInputNormals(cloud_normals);
    // alternatively, if cloud is of type PointNormal, do vfh.setInputNormals (cloud)

    // Use the same kdtree when estimating normals since we do not use its content
    // in this example
    vfh.setSearchMethod(tree);
    span = timer.Stop();
    cout << "VFH Estimation preparation time (ms): " << span << endl;

    // Output
    pcl::PointCloud<pcl::VFHSignature308>::Ptr 
        vfhs(new pcl::PointCloud<pcl::VFHSignature308>);

    timer.Start();
    vfh.compute(*vfhs);
    span = timer.Stop();
    cout << "VFH Estimation time (ms): " << span << endl;

    return EXIT_SUCCESS;
}
