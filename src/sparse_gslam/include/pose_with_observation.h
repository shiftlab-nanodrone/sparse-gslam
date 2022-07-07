#pragma once
#include "cartographer_bindings/range_data_2d.h"
#include "g2o/types/slam2d/edge_se2.h"
#include "g2o/types/slam2d/vertex_se2.h"
#include "delta_vector.h"
using namespace cartographer;
class PoseWithObservation;
using PoseWithObservations = std::deque<PoseWithObservation, Eigen::aligned_allocator<PoseWithObservation>>;
class PoseWithObservation {
   public:
    g2o::VertexSE2 pose;
    g2o::EdgeSE2 edge;
    sensor::RangeData2D data;
    DeltaVector odom;
    PoseWithObservation(const sensor_msgs::LaserScan& scan, const std::vector<Eigen::Vector2f>& table, const g2o::SE2& raw_odom);
    void addPoints(const sensor_msgs::LaserScan& scan, const g2o::SE2& transform, const std::vector<Eigen::Vector2f>& table);
    static void construct_multicloud_with_correct(
        const PoseWithObservations& poses, 
        int start, int mid, int end, sensor::RangeData2D& result);
    static void construct_multicloud_with_correct_returns_only(
        const PoseWithObservations& poses, 
        int start, int mid, int end, std::vector<Eigen::Vector2f>& result);
};