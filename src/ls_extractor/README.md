# Line Segment Extractor

Line segment extractor for 2D pointclouds/LiDARs data with error propagation. Similar to https://github.com/kam3k/laser_line_extraction, but with the following differences

1. Core implementation (in [include/impl](include/impl) and [src/impl](src/impl)) can be compiled without ros
2. Error propagation model is different: we allow users to specify difference covariance matrices for each point
3. Agglomerative clustering can be optionally enabled before line segment extraction

By default, the split-and-merge with clustering (smc) algorithm is used to perform line segment extraction because of its efficiency and performance. However, an implementation of prototype based fuzzing (also named as split-and-merge fuzzing, smf) is provided. If you wish to test this implementation, change includes of "smc.h" to "smf.h" and link against "smf" instead. 

## Configurations

See [config/example.yaml](config/example.yaml) for all configurable parameters and their meaning

## Launch files

An example ROS node that subscribes to LaserScan together with a launch file is provided. See (src/ls_extractor/launch/laser.launch)[src/ls_extractor/launch/laser.launch]. Note that it puts scaled identity matrices as the covariance for each point. You can adapt the code for your need. 

```bash
roslaunch ls_extractor laser.launch
```