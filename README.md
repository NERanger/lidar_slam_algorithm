# LiDAR SLAM Algorithms

LiDAR SLAM related algorithms mainly for myself. Currently all pointclouds used for test are from KITTI dataset. New data loader need to be implemented if a new dataset is used.

## Environment

Ubuntu 18.04 LTS (WSL2)

## Requirements

[PCL(Point Cloud Library)](https://pcl.readthedocs.io/projects/tutorials/en/latest/index.html)

## Compile

```shell
mkdir build
cd build
cmake ..
make
```

## Documentation

See [docs](./docs)