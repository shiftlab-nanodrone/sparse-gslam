#pragma once
#include <unordered_map>

#include "ls_extractor/defs.h"
namespace ls_extractor {
using namespace std;

class LineSegment : public _LineSegment {
   public:
    LineSegment() {}
    LineSegment(pit start, pit end) : start(start), end(end) {
    }

   private:
    pit start, end;
    void endpointFit();
    float gapBetween(const LineSegment& other) const;
    void leastSqFit();
    void projectEndpoints();
    inline float length() const {
        return (start->point - (end - 1)->point).norm();
    }
    friend class LineSegmentExtractor;
};
// TODO: in theory we need an aligned allocator here, but it will cause a segfault when the destructor is called. Not sure why.
using SegmentVector = vector<LineSegment>;
class LineSegmentExtractor {
   public:
    SegmentVector segments;
    Params params;
    vector<pit> splits;

    void extract_lines(PointVector& points);

   private:
    void extract_lines_helper(PointVector& points);
    void split(pit start, pit end);
    void merge();
    void merge_pairwise();
    unordered_map<int, PointVector> cluster(const PointVector& points);

    vector<int> ranks, parents;
};
}  // namespace ls_extractor