#pragma once
#include "g2o/core/sparse_optimizer.h"
#include "g2o/types/slam2d/edge_se2.h"
#include "g2o/types/slam2d/vertex_se2.h"
#include "g2o_bindings/edge_se2_rhotheta.h"
#include "g2o_bindings/vertex_rhotheta.h"
#include "pose_with_observation.h"
#include <boost/thread.hpp>
#include <deque>
#include <unordered_set>

void setup_lm_opt(g2o::SparseOptimizer& opt);
void setup_pose_opt(g2o::SparseOptimizer& opt);

struct PoseChain {
    g2o::VertexSE2 pose;
    g2o::EdgeSE2 edge;
};
struct LandmarkGraph {
    int last_landmark_edge = 0;
    boost::shared_mutex mu;
    std::deque<g2o::VertexRhoTheta, Eigen::aligned_allocator<g2o::VertexRhoTheta>> landmarks;
    std::deque<g2o::EdgeSE2RhoTheta, Eigen::aligned_allocator<g2o::EdgeSE2RhoTheta>> pose_landmarks;
    std::deque<PoseWithObservation, Eigen::aligned_allocator<PoseWithObservation>> poses;
    g2o::HyperGraph::VertexSet new_vset;
    g2o::HyperGraph::EdgeSet new_eset;
    g2o::SparseOptimizer opt;
    LandmarkGraph();
    ~LandmarkGraph();
};
struct PoseGraph {
    boost::shared_mutex mu;
    std::deque<PoseChain, Eigen::aligned_allocator<PoseChain>> poses;
    std::deque<g2o::EdgeSE2, Eigen::aligned_allocator<g2o::EdgeSE2>> all_closures;
    std::unordered_set<g2o::EdgeSE2*> closures, false_closures;
    std::vector<g2o::EdgeSE2*> false_candidates;
    g2o::SparseOptimizer opt;
    PoseGraph();
    ~PoseGraph();
};