# Simple SLAM

### 1. Monocular Visual Odometry

This uses data from the [KITTI Visual Odometry Challenge](https://www.cvlibs.net/datasets/kitti/eval_odometry.php).

The goal of this step is to obtain initial poses and landmarks relative to the poses.

### 2. SLAM

The KITTI data contains loop closures which can be used to close the factor graph and optimize the entire path. Using the optimized poses, a map of the landmarks can be build.

This map could then be used downstream to plan new paths in the environment.

## Build

```
cmake -S . -B build
cmake --build build -j4
```