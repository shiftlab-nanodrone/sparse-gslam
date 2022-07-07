# Efficient Graph SLAM for Sparse Sensing

## Installation

We recommend Ubuntu 20.04 with ROS noetic, but it should also work on Ubuntu 18.04 with ROS melodic. Our code requires a C++14 capable compiler. 

## Prereqs

1. Install dependencies using package manager:

```
sudo apt install ros-noetic-jsk-rviz-plugins ros-noetic-navigation ros-noetic-joy
```

2. Follow the instruction on https://google-cartographer.readthedocs.io/en/latest/ to build and install google cartographer

3. Clone the libg2o-release repository (https://github.com/ros-gbp/libg2o-release) and
    > We use this repo rather than the latest g2o because g2o comes with its own version of ceres-solver, which may conflict with the version used by cartographer. This makes sure that they will not conflict.
    - checkout release/{your-ros-distro}/libg2o branch
    - in config.h.in, make sure that the macro `#define G2O_DELETE_IMPLICITLY_OWNED_OBJECTS 0` is enabled
    > we manage memories for edges and vertices ourselves. You will get segfault if you miss this step.
    - in CMakeLists.txt, search for `BUILD_WITH_MARCH_NATIVE` and make sure it is ON
    > You may get segfault if you miss this step
    - follow the readme to build and install it
    - run `sudo ldconfig` command to update links

4. Clone this repository and build

```bash
catkin_make -DCMAKE_BUILD_TYPE=Release
```

5. Download datasets and evaluation scripts by running

```bash
cd src/sparse_gslam/datasets
./download.sh
```

## Running our code

```bash
roslaunch sparse_gslam log_runner.launch dataset:={dataset_name}
```

where {dataset_name} is one of the directory names under src/sparse_gslam/datasets (e.g. intel-lab).

## Compute SLAM Metrics

For aces, intel-lab and mit-killian, ground truths are available and SLAM metrics can be calculated. After running our code with the command above and waiting for it to finish, you can use our evaluation script to compute the metrics

```bash
cd src/sparse_gslam/datasets
./eval.sh {dataset_name}
```

