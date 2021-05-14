#include <iostream>
#include <string>
#include <fstream>
#include <vector>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/features/normal_3d.h>

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

    timer.Start();
    NormalEstimation<PointXYZI, Normal> n;
    n.setInputCloud(ptcloud);

    KdTree<PointXYZI>::Ptr tree(new KdTree<PointXYZI>());
    n.setSearchMethod(tree);

    PointCloud<Normal>::Ptr cloud_normals(new PointCloud<Normal>);
    n.setRadiusSearch(0.05); // Estimation based on points within 5cm radius
    n.compute(*cloud_normals);
    span = timer.Stop();
    cout << "Normal Estimation time (ms): " << span << endl;

    return EXIT_SUCCESS;
}