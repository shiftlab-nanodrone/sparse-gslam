#pragma once
#include <g2o/types/slam2d/se2.h>

#include <Eigen/StdVector>
struct Delta {
    double dt;
    g2o::SE2 dpose;
};

typedef std::vector<Delta, Eigen::aligned_allocator<Delta>> DeltaVector;