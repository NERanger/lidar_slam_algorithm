# Estimating VFH signatures

Estimating VFH signatures for a set of points. This feature is intended for Cluster (e.g., Object) Recognition and 6DOF Pose Estimation. Therefore, applying this feature on KITTI pointcloud may be pointless.

## Usage

Exxecute the following shell script in root directory of this repo after compiling

```shell
./bin/vfh_extraction <path-to-pointcloud>
```

## Reference

<https://pcl.readthedocs.io/projects/tutorials/en/latest/vfh_estimation.html#vfh-estimation>