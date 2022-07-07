#pragma once
#include "cartographer_bindings/fast_correlative_scan_matcher_2d.h"
#include "cartographer_bindings/ceres_scan_matcher_2d.h"
#include "cartographer_bindings/range_data_inserter_2d.h"
#include "ctpl_stl.h"
#include "submap.h"

class Drone;
class SubmapLoopCloser {
   public:
    Drone& drone;

    double loop_closure_min_score;
    const double max_match_distance;
    const double last_traj_length;
    const double submap_resolution;
    const double submap_trajectory_length;
    const int last_submap_not_match;
    const int submap_overlap_poses;
    int last_pose_idx = 0;
    int last_opt_pose_index = 1;

    struct SubmapInfo {
        float x, y, theta;
        Submap* submap;
    };
    std::vector<SubmapInfo> submaps_info;
    std::deque<Submap> submaps;
    std::vector<Eigen::Vector2f> multi_scan;

    mapping::scan_matching::proto::FastCorrelativeScanMatcherOptions2D matcher_options;
    mapping::MultirangeDataInserter range_data_inserter;
    mapping::scan_matching::CeresScanMatcher2D ceres_matcher;

    SubmapLoopCloser(Drone& visualizer, const XmlRpc::XmlRpcValue& config);
    void precompute();
    bool match();

    ctpl::thread_pool pool;
};