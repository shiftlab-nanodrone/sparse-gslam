#pragma once
#include <Eigen/Core>
#include <Eigen/StdVector>
#include <vector>

namespace ls_extractor {
using namespace std;

// note that we store both the Cartesian and polar coordinate 
// because we use both form to perform computations
struct Point {
    Eigen::Vector2f point; // (x, y) coordinate
    // (rho, theta) polar coordinate
    // equals to (sqrt(x^2+y^2), atan2(y, x))
    Eigen::Vector2f rhotheta;
    // covariance matrix of the (x, y) coordinate
    Eigen::Matrix2f cov;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
};

struct Params {
    float outlier_dist = 0.1;
    float min_split_dist = 0.15;
    float max_line_gap = 0.25;
    float min_line_length = 0.5;
    float rmse_thresh = 0.1;
    float cluster_threshold = 0.5;
    int min_line_points = 10;
};

struct _LineSegment {
  public:
    Eigen::Vector2f start_, end_;
    Eigen::Vector2f rhotheta;
    Eigen::Matrix2f cov;
  protected:
    inline float distToPoint(const Point& point) const {
        return abs(point.rhotheta[0] * cos(point.rhotheta[1] - rhotheta[1]) - rhotheta[0]);
    }
};

using PointVector = vector<Point, Eigen::aligned_allocator<Point>> ;
using pit = PointVector::const_iterator;
using PPtrVector = vector<const Point*>;
}