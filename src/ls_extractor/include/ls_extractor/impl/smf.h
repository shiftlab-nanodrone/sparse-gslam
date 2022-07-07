#pragma once
#include <Eigen/Core>
#include <ostream>

#include "ls_extractor/defs.h"

namespace ls_extractor {
using namespace std;

float pairwise_closest(const vector<const Point*>& points1, const vector<const Point*>& points2);

struct LineSegment : public _LineSegment {
    vector<const Point*> points;
    vector<float> uj, dj;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    LineSegment() = default;
    LineSegment(const PointVector& points);
    LineSegment(const vector<const Point*>& points);

   private:
    float calc_xybar(Eigen::Vector2f& xybar, float m) const;
    void calc_dij(const Eigen::Vector2f& xybar);
    float calc_dispersion() const;

    void leastSqFit(const float m, bool calc_dij_flag = true, bool calc_cov = false);
    void projectEndpoints();

    bool satisfy_param(float max_gap, float min_length) const;
    friend class LineSegmentExtractor;
};
using SegmentVector = std::vector<LineSegment>;
class LineSegmentExtractor {
   public:
    SegmentVector segments;
    Params params;

    void extract_lines(PointVector& points);

    void merge();
    void merge2();

    // prototype based fuzzying
    void PBF(LineSegment& segment, float m = 2.0f);
};
}  // namespace ls_extractor