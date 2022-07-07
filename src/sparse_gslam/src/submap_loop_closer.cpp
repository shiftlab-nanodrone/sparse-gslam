#include "loop_closer/submap_loop_closer.h"

#include <fstream>
#include <iostream>
#include <string>

#include "drone.h"
#include "g2o/core/robust_kernel_impl.h"
#include "g2o/types/slam2d/edge_se2.h"
#include "pose_with_observation.h"
#include "g2o_bindings/edge_se2_rhotheta.h"

#ifdef SHOW_MATCH
#include "ls_extractor/matplotlibcpp.h"
namespace plt = matplotlibcpp;
#endif

// ceres_scan_matcher = {
//   occupied_space_weight = 20.,
//   translation_weight = 10.,
//   rotation_weight = 1.,
//   ceres_solver_options = {
//     use_nonmonotonic_steps = true,
//     max_num_iterations = 10,
//     num_threads = 1,
//   },
// },

// this is copied from cartographer's default options
auto get_ceres_options(const XmlRpc::XmlRpcValue& config) {
    mapping::scan_matching::proto::CeresScanMatcherOptions2D opt;
    opt.set_occupied_space_weight(20);
    opt.set_translation_weight(10);
    opt.set_rotation_weight(1);
    opt.mutable_ceres_solver_options()->set_use_nonmonotonic_steps(true);
    opt.mutable_ceres_solver_options()->set_max_num_iterations(10);
    opt.mutable_ceres_solver_options()->set_num_threads(1);
    return opt;
}

g2o::RobustKernelDCS dcs_kernel;

SubmapLoopCloser::SubmapLoopCloser(Drone& drone, const XmlRpc::XmlRpcValue& config) : drone(drone),
                                                                                      loop_closure_min_score(config["loop_closure_min_score"]),
                                                                                      max_match_distance(config["max_match_distance"]),
                                                                                      last_traj_length(config["last_traj_length"]),
                                                                                      submap_resolution(config["submap_resolution"]),
                                                                                      submap_trajectory_length(config["submap_trajectory_length"]),
                                                                                      last_submap_not_match(config["last_submap_not_match"]),
                                                                                      submap_overlap_poses(config["submap_overlap_poses"]),
                                                                                      range_data_inserter(config["hit_probability"], config["miss_probability"]),
                                                                                      ceres_matcher(get_ceres_options(config)),
                                                                                      pool(config["loop_closing_threads"]) {
    matcher_options.set_angular_search_window(config["angular_search_window"]);
    matcher_options.set_linear_search_window(config["linear_search_window"]);
    matcher_options.set_branch_and_bound_depth(config["branch_and_bound_depth"]);
    dcs_kernel.setDelta(config["dcs_phi"]);
    #ifdef SHOW_MATCH
    // plt::ion();
    #endif
}

void SubmapLoopCloser::precompute() {
    boost::shared_lock<boost::shared_mutex> lock(drone.lm_graph.mu);
    if (!drone.lm_graph.poses.size())
        return;

    double traj_length = 0;
    int i, mid = -1;
    for (i = last_pose_idx + 1; i < drone.lm_graph.poses.size(); i++) {
        traj_length += (drone.lm_graph.poses[i].pose.estimate().translation() - drone.lm_graph.poses[i - 1].pose.estimate().translation()).norm();
        if (traj_length > submap_trajectory_length / 2 && mid == -1)
            mid = i;
        if (traj_length > submap_trajectory_length)
            break;
    }
    if (traj_length > submap_trajectory_length) {
        sensor::RangeData2D temp_range;
        PoseWithObservation::construct_multicloud_with_correct(drone.lm_graph.poses, last_pose_idx, mid, i + 1, temp_range);
        lock.unlock();

        std::cout << "finishing a new submap with " << temp_range.points_.size() << std::endl;
        submaps.emplace_back(0, submap_resolution);
        auto& submap = submaps.back();
        {
            range_data_inserter.Insert(temp_range, &submap.probability_grid);
            auto c_grid = submap.probability_grid.ComputeCroppedGrid();
            submap.probability_grid = std::move(*static_cast<mapping::ProbabilityGrid*>(c_grid.get()));
        }
        {
            range_data_inserter.Insert(temp_range, &submap.high_res_grid);
            auto c_grid = submap.high_res_grid.ComputeCroppedGrid();
            submap.high_res_grid = std::move(*static_cast<mapping::ProbabilityGrid*>(c_grid.get()));
        }
        submap.fix_submap(&drone.lm_graph.poses[mid].pose, matcher_options, 0.5f);
        last_pose_idx = std::max(0, mid - submap_overlap_poses);
    }

    // for (auto& sub : submaps) {
    //     if (sub.matcher.is_initialized()) {
    //         sub.publish_without_compute(ros::Time::now());
    //     }
    // }
}

struct MatchResult {
    bool match;
    float score;
    Submap* it;
    transform::Rigid2d pose_estimate;
    Eigen::Matrix3d covariance;
};

MatchResult matchOne(int id, Submap* it, double angle_est, const std::vector<Eigen::Vector2f>& c_pc, const double loop_closure_min_score) {
    MatchResult result;
    result.it = it;
    result.match = it->matcher.get().Match(
        transform::Rigid2d::Rotation(angle_est), c_pc, loop_closure_min_score, 
        &result.score, &result.pose_estimate, result.covariance);
    return result;
}

int num_matches = 0;
bool SubmapLoopCloser::match() {
    double traj_length = 0;
    int i, mid = -1;

    boost::shared_lock<boost::shared_mutex> lm_lock(drone.lm_graph.mu);
    int num_poses = drone.lm_graph.poses.size();
    if (num_poses <= 2 || submaps.size() <= last_submap_not_match)
        return false;
    // note here we exclude the potentially incomplete observation from the last pose
    for (i = num_poses - 2; i >= 0; i--) {
        traj_length += (drone.lm_graph.poses[i + 1].pose.estimate().translation() - drone.lm_graph.poses[i].pose.estimate().translation()).norm();
        if (traj_length >= last_traj_length / 2 && mid == -1) {
            if (i - last_opt_pose_index < 2)
                return false;
            mid = i;
        }
        if (traj_length >= last_traj_length)
            break;
    }
    // -------------- copy stuff to avoid race while holding the lock ----------------------------
    // deep copy last several poses
    PoseWithObservation::construct_multicloud_with_correct_returns_only(drone.lm_graph.poses, i, mid, drone.lm_graph.poses.size(), multi_scan);

    // copy the submaps poses
    submaps_info.clear();
    auto eit = submaps.end() - last_submap_not_match;
    auto trans_pre = drone.pose_graph.poses[drone.loop_closer.last_opt_pose_index - 1].pose.estimate() *
                     drone.lm_graph.poses[drone.loop_closer.last_opt_pose_index - 1].pose.estimate().inverse();
    auto bl_trans = mid < drone.loop_closer.last_opt_pose_index ? drone.pose_graph.poses[mid].pose.estimate() : trans_pre * drone.lm_graph.poses[mid].pose.estimate();
    for (auto it = submaps.begin(); it != eit; it++) {
        auto map_pose = it->pose->id() < drone.loop_closer.last_opt_pose_index ? static_cast<g2o::VertexSE2*>(drone.pose_graph.opt.vertex(it->pose->id()))->estimate() : trans_pre * it->pose->estimate();
        if ((bl_trans.translation() - map_pose.translation()).norm() >= max_match_distance)
            continue;
        submaps_info.push_back({(float)map_pose[0],
                                (float)map_pose[1],
                                (float)map_pose[2],
                                &(*it)});
    }
    // -------------- end ---------------------------------------------------------------
    lm_lock.unlock();
    std::vector<std::future<MatchResult>> results;
    results.reserve(submaps_info.size());
    for (const auto& info : submaps_info) {
        results.push_back(pool.push(matchOne, info.submap, g2o::normalize_theta(bl_trans[2] - info.theta), multi_scan, loop_closure_min_score));
    }

    MatchResult b_result;
    b_result.match = false;
    b_result.score = 0.0;
    for (auto& f : results) {
        const auto& r = f.get();
        if (r.match && r.score > b_result.score) {
            b_result = r;
        }
    }
    if (b_result.match) {
        std::cout << "match score: " << b_result.score << '\n';
        {
            sensor::PointCloud pc;
            for (const auto& pt : multi_scan) {
                pc.push_back({{pt[0], pt[1], 0.0f}});
            }
            ceres::Solver::Summary unused_summary;
            ceres_matcher.Match(b_result.pose_estimate.translation(), b_result.pose_estimate, pc,
                                b_result.it->high_res_grid, &b_result.pose_estimate,
                                &unused_summary);
        }
#ifdef SHOW_MATCH
        auto& grid = b_result.it->grid;
        std::cout << "match transform: " << transform::ToProto(b_result.pose_estimate).DebugString();
        std::vector<float> x, y;
        x.reserve(multi_scan.size());
        y.reserve(multi_scan.size());
        auto _pef = b_result.pose_estimate.cast<float>();
        for (auto& p : multi_scan) {
            Eigen::Vector2f pi = (_pef * p - b_result.it->box.min()) / b_result.it->grid.info.resolution;
            x.push_back(pi[0]);
            y.push_back(pi[1]);
        }
        plt::clf();
        plt::imshow((unsigned char*)grid.data.data(), grid.info.height, grid.info.width, 1, {{"cmap", "gray"}});
        plt::scatter(x, y, 2.0);

        std::stringstream title_ss;
        title_ss << "accepted\n";
#endif
        {
            lm_lock.lock();
            boost::unique_lock<boost::shared_mutex> pose_lock(drone.pose_graph.mu);
            auto* prev_vertex = &drone.pose_graph.poses.back().pose;
            for (auto it = drone.lm_graph.poses.begin() + last_opt_pose_index; it != drone.lm_graph.poses.end(); it++) {
                drone.pose_graph.poses.emplace_back();
                auto* pose = &drone.pose_graph.poses.back().pose;
                auto* edge = &drone.pose_graph.poses.back().edge;
                pose->setId(it->pose.id());

                edge->vertices()[0] = prev_vertex;
                edge->vertices()[1] = pose;
                edge->information() = it->edge.information();
                edge->setMeasurement((it - 1)->pose.estimate().inverse() * it->pose.estimate());
                // edge->setMeasurement(it->edge.measurement());
                pose->setEstimate(prev_vertex->estimate() * edge->measurement());
                drone.pose_graph.opt.addVertex(pose);
                drone.pose_graph.opt.addEdge(edge);

                prev_vertex = pose;
            }
            last_opt_pose_index = drone.lm_graph.poses.size();
            pose_lock.unlock();
            lm_lock.unlock();

            // {
            //     boost::unique_lock<boost::shared_mutex> lm_unique_lock(drone.lm_graph.mu);
            //     auto it = drone.lm_graph.poses.begin() + std::max(1, last_opt_pose_index - 1);

            //     std::vector<g2o::EdgeSE2RhoTheta*> poseLandmarks;
            //     std::unordered_set<g2o::VertexRhoTheta*> landmarks;
            //     poseLandmarks.reserve(it->pose.edges().size() - 1);
            //     for (auto* edge : it->pose.edges()) {
            //         if (dynamic_cast<g2o::EdgeSE2RhoTheta*>(edge)) {
            //             poseLandmarks.push_back(static_cast<g2o::EdgeSE2RhoTheta*>(edge));
            //             landmarks.insert(static_cast<g2o::VertexRhoTheta*>(edge->vertex(1)));
            //         }
            //     }

            //     drone.lm_graph.opt.clear();
            //     it->pose.setFixed(true);
            //     drone.lm_graph.opt.addVertex(&it->pose);
            //     for (auto* lm : landmarks) {
            //         // lm->setFixed(true);
            //         drone.lm_graph.opt.addVertex(lm);
            //     }
            //     for (auto* edge : poseLandmarks) {
            //         drone.lm_graph.opt.addEdge(edge);
            //     }
            //     drone.need_reinit = true;
            // }

            {
                boost::unique_lock<boost::shared_mutex> lm_unique_lock(drone.lm_graph.mu);
                auto it = drone.lm_graph.poses.begin() + std::max(1, last_opt_pose_index - 1);
                drone.lm_graph.opt.clear();
                it->pose.setFixed(true);
                drone.lm_graph.opt.addVertex(&it->pose);
                drone.need_reinit = true;

                // can remove old landmarks for better performance
                // int prev_size = drone.lm_graph.landmarks.size();
                // while (drone.lm_graph.landmarks.size() && drone.traveled_dist > drone.lm_graph.landmarks.front().dist + drone.landmark_max_dist) {
                //     drone.lm_graph.landmarks.pop_front();
                // }
                // std::cout << "removed " << drone.lm_graph.landmarks.size() - prev_size << " old landmarks\n";
            }

            pose_lock.lock();
            drone.pose_graph.all_closures.emplace_back();
            auto* odom_edge = &drone.pose_graph.all_closures.back();
            odom_edge->setMeasurement(g2o::SE2(b_result.pose_estimate.translation()[0], b_result.pose_estimate.translation()[1], b_result.pose_estimate.rotation().angle()));
            odom_edge->information().noalias() = b_result.covariance.inverse();
#ifdef SHOW_MATCH
            title_ss << odom_edge->information();
#endif
            odom_edge->vertices()[0] = drone.pose_graph.opt.vertices()[b_result.it->pose->id()];
            odom_edge->vertices()[1] = &drone.pose_graph.poses[mid].pose;

            odom_edge->setRobustKernel(&dcs_kernel);
            drone.pose_graph.closures.insert(odom_edge);
            drone.pose_graph.opt.addEdge(odom_edge);
            drone.pose_graph.opt.initializeOptimization();
            drone.pose_graph.opt.optimize(20);
            drone.pose_graph.opt.computeActiveErrors();
        }

        // plt::title(title_ss.str());
        // plt::pause(0.00001);
        // plt::save("match_" + std::to_string(num_matches++) + "_" + std::to_string(b_result.score) + ".png");
        return true;
    }
    return false;
}
