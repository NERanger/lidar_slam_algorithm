#include <iostream>
#include <string>
#include <fstream>
#include <vector>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/pfh.h>

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
using pcl::PFHEstimation;
using pcl::PFHSignature125;

// Ref: https://pcl.readthedocs.io/projects/tutorials/en/latest/pfh_estimation.html#pfh-estimation

int main(int argc, char const *argv[]){
    if(argc != 2){
        cout << "Usage: PFH_extraction <path-to-pointcloud>" << endl;
        return EXIT_FAILURE;
    }

    Timer timer;
    int64_t span = 0;

    timer.Start();
    string path(argv[1]);
    PointCloud<PointXYZI>::Ptr ptcloud = KittiLoader::LoadSingleFrame(path);
    span = timer.Stop();
    cout << "Frame load time (ms): " << span << endl;

    timer.Start();
    KdTree<PointXYZI>::Ptr tree(new KdTree<PointXYZI>());

    NormalEstimation<PointXYZI, Normal> n;
    n.setInputCloud(ptcloud);
    n.setSearchMethod(tree);

    PointCloud<Normal>::Ptr cloud_normals(new PointCloud<Normal>);
    n.setRadiusSearch(0.05); // Estimation based on points within 5cm radius
    span = timer.Stop();
    cout << "Normal Estimation preparation time (ms): " << span << endl;

    timer.Start();
    n.compute(*cloud_normals);
    span = timer.Stop();
    cout << "Normal Estimation time (ms): " << span << endl;

    timer.Start();
    // Create the PFH estimation class, and pass the input dataset+normals to it
    PFHEstimation<PointXYZI, Normal, pcl::PFHSignature125> pfh;
    pfh.setInputCloud(ptcloud);
    pfh.setInputNormals(cloud_normals);
    // alternatively, if cloud is of type PointNormal, do pfh.setInputNormals (cloud);

    // Create an empty kdtree representation, and pass it to the PFH estimation object.
    // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
    pfh.setSearchMethod(tree);

    PointCloud<PFHSignature125>::Ptr pfhs(new PointCloud<PFHSignature125>());
    
    // Use all neighbors in a sphere of radius 10cm
    // IMPORTANT: the radius used here has to be larger than the radius used to estimate the surface normals!!!
    pfh.setRadiusSearch(0.10);
    span = timer.Stop();
    cout << "PFH computation preparation time (ms): " << span << endl;

    timer.Start();
    // Compute the features
    pfh.compute (*pfhs);
    span = timer.Stop();
    cout << "Full pointcloud PFH computation time (ms): " << span << endl;

    return EXIT_SUCCESS;
}