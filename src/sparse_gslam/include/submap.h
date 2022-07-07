#pragma once
#include <nav_msgs/OccupancyGrid.h>
#include <ros/ros.h>
#include <boost/optional.hpp>

#include <unordered_set>

#include "cartographer/mapping/2d/probability_grid.h"
#include "cartographer_bindings/fast_correlative_scan_matcher_2d.h"
#include "cartographer_bindings/range_data_2d.h"

namespace g2o {
    class VertexSE2;
    class VertexRhoTheta;
    class SE2;
};
using namespace cartographer;
struct Submap {
    static mapping::ValueConversionTables conversion_tables;
    static Eigen::AlignedBox2f probability_grid_to_occupancy_grid(
        const g2o::SE2& trans, const mapping::ProbabilityGrid& probability_grid, nav_msgs::OccupancyGrid& grid, float padding);
    static void probability_grid_to_occupancy_grid_fixed_size(
        const mapping::ProbabilityGrid& probability_grid, nav_msgs::OccupancyGrid& grid);

    int submap_idx;
    g2o::VertexSE2* pose;
    Eigen::AlignedBox2f box;
    ros::Publisher pub;
    nav_msgs::OccupancyGrid grid;
    mapping::ProbabilityGrid probability_grid;
    mapping::ProbabilityGrid high_res_grid;
    // delayed initialization for matcher. it will only be constructed after the submap is finalized
    boost::optional<mapping::scan_matching::FastCorrelativeScanMatcher2D> matcher;
    Submap(const Submap& a) = delete;
    Submap(Submap&& a) = default;
    Submap(int idx, float res);
    void fix_submap(g2o::VertexSE2* pose, const mapping::scan_matching::proto::FastCorrelativeScanMatcherOptions2D& options, float padding);
    void publish_without_compute(ros::Publisher& pub);
};