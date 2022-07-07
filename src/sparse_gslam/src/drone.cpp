#include "drone.h"

#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include <tf2_ros/transform_listener.h>

#include <boost/math/distributions/chi_squared.hpp>
#include <unordered_set>

#include "g2o_bindings/edge_se2_rhotheta.h"
#include "ls_extractor/impl/smc.h"
#include "ls_extractor/utils.h"

using chi2_distribution = boost::math::chi_squared_distribution<double>;

double norm_angle(double angle) {
    if (angle > M_PI) {
        return angle - 2 * M_PI;
    } else if (angle < -M_PI) {
        return angle + 2 * M_PI;
    } else {
        return angle;
    }
}

Drone::Drone(ros::NodeHandle &nh,
             const XmlRpc::XmlRpcValue &config) : odom_prop(config["std_x"], config["std_y"], config["std_w"]),
                                                  cor_pose_pub(nh.advertise<geometry_msgs::PoseStamped>("/corrected_pose", 1)),
                                                  loop_closer(*this, config),
                                                  landmark_assoc_thresh(config["landmark_assoc_thresh"]),
                                                  landmark_max_gap(config["landmark_max_gap"]),
                                                  landmark_max_dist(config["landmark_max_dist"]) {
    cor_pose_msg.header.frame_id = "map";
    odom_bl_tf.header.frame_id = "odom";
    odom_bl_tf.child_frame_id = "base_link";

    tf2_ros::Buffer tf_buffer;
    tf2_ros::TransformListener tf_listener(tf_buffer);
    if (!tf_buffer.canTransform("map", "odom", ros::Time(0), ros::Duration(2.0)))
        abort();
    tf::transformMsgToTF(tf_buffer.lookupTransform("map", "odom", ros::Time(0)).transform, odom_map_tf);
}

void Drone::msgCallback(const ls_extractor::SegmentVector &bl_lines, const nav_msgs::Odometry &odom, const sensor_msgs::LaserScan &pc) {
    tf::Transform transform;
    tf::poseMsgToTF(odom.pose.pose, transform);
    transform = odom_map_tf * transform;
    auto &_translation = transform.getOrigin();
    auto &_mat = transform.getBasis();
    double _r, _p, _y;
    _mat.getRPY(_r, _p, _y);
    g2o::SE2 new_pose(_translation.getX(), _translation.getY(), _y);

    if (!lm_graph.poses.size()) {  // add initial fixed pose
        table.resize(pc.ranges.size());
        for (int i = 0; i < pc.ranges.size(); i++) {
            float angle = pc.angle_min + pc.angle_increment * i;
            table[i] = {cos(angle), sin(angle)};
        }

        boost::unique_lock<boost::shared_mutex> lm_lock(lm_graph.mu);
        lm_graph.poses.emplace_back(pc, table, new_pose);
        auto *initialPose = &lm_graph.poses.back().pose;
        initialPose->setId(vertex_id++);
        initialPose->setEstimate(new_pose);
        initialPose->setFixed(true);
        lm_graph.opt.addVertex(initialPose);
        lm_graph.new_vset.insert(initialPose);

        boost::unique_lock<boost::shared_mutex> pose_lock(pose_graph.mu);
        pose_graph.poses.emplace_back();
        auto *ip = &pose_graph.poses.back().pose;
        ip->setId(initialPose->id());
        ip->setEstimate(initialPose->estimate());
        ip->setFixed(true);
        pose_graph.opt.addVertex(ip);
        prev_pose.dt = odom.header.stamp.toSec();
        prev_pose.dpose = new_pose;
        return;
    }

    // compute the delta pose and update prev pose
    auto delta_pose = prev_pose.dpose.inverse() * new_pose;
    odom_prop.step({odom.header.stamp.toSec() - prev_pose.dt, delta_pose});
    prev_pose.dt = odom.header.stamp.toSec();
    prev_pose.dpose = new_pose;

    // compute the corrected pose based on the last optimize pose vertex
    auto *prev_vertex = &lm_graph.poses.back().pose;
    g2o::SE2 cor_pose = prev_vertex->estimate() * odom_prop.pose;

    // update transform so it has the correction applied
    _translation.setX(cor_pose[0]);
    _translation.setY(cor_pose[1]);
    _mat.setRPY(_r, _p, cor_pose[2]);

    odom_bl_tf.header.stamp = odom.header.stamp;
    tf::transformTFToMsg(odom_map_tf.inverse() * transform, odom_bl_tf.transform);
    br.sendTransform(odom_bl_tf);

    // ---------------- visualization--------------------
    cor_pose_msg.header.stamp = odom.header.stamp;
    cor_pose_msg.pose.position.x = _translation.getX();
    cor_pose_msg.pose.position.y = _translation.getY();
    cor_pose_msg.pose.position.z = _translation.getZ();
    tf::quaternionTFToMsg(transform.getRotation(), cor_pose_msg.pose.orientation);
    cor_pose_pub.publish(cor_pose_msg);
    // ---------------- end visualization--------------------

    // update error with the computed delta
    double disp = odom_prop.pose.translation().norm();
    if ((disp > 0.5 || std::abs(odom_prop.pose[2]) >= M_PI / 6)) {
        traveled_dist += disp;

        boost::unique_lock<boost::shared_mutex> lm_lock(lm_graph.mu);
        lm_graph.poses.emplace_back(pc, table, new_pose);
        auto *new_vertex = &lm_graph.poses.back().pose;
        auto *odom_edge = &lm_graph.poses.back().edge;

        new_vertex->setEstimate(cor_pose);
        new_vertex->setId(vertex_id++);
        lm_graph.opt.addVertex(new_vertex);
        lm_graph.new_vset.insert(new_vertex);

        odom_edge->vertices()[0] = prev_vertex;
        odom_edge->vertices()[1] = new_vertex;
        odom_edge->setMeasurement(odom_prop.pose);
        odom_edge->information().noalias() = odom_prop.cov.inverse();
        // std::cout << "pose information " << std::endl;
        // std::cout << odom_edge->information() << std::endl;
        lm_graph.opt.addEdge(odom_edge);
        lm_graph.new_eset.insert(odom_edge);

        Eigen::Vector2f trans = cor_pose.translation().cast<float>();
        Eigen::Rotation2Df rot = cor_pose.rotation().cast<float>();
        for (const auto &line : bl_lines) {
            // transform the observed segments to the global frame
            auto *landmark = mergeLine(
                rot * line.start_ + trans,
                rot * line.end_ + trans);
            addLandmarkObservations(landmark, line, new_vertex);
        }
        odom_prop.reset();

        // online optimization is kind of buggy. If it results in segfault, please disable it
        if (need_reinit) {
            // we cannot rely on updateInitialization when we removed some edges or it is the first time we call init/optimize
            lm_graph.opt.initializeOptimization();
            lm_graph.opt.push();
            lm_graph.opt.optimize(15, false);
            need_reinit = false;
        } else {
            lm_graph.opt.updateInitialization(lm_graph.new_vset, lm_graph.new_eset);
            lm_graph.opt.push();
            lm_graph.opt.optimize(15, true);
        }

        // later when we need to remove edges because of rejection, this makes sure that we keep the pose-pose edge
        lm_graph.new_eset.erase(odom_edge);

        int dof = 0;
        for (auto *edge : lm_graph.opt.activeEdges())
            dof += static_cast<g2o::OptimizableGraph::Edge *>(edge)->dimension();
        lm_graph.opt.computeActiveErrors();
        double chi2_after = lm_graph.opt.activeChi2();
        // std::cout << chi2_before << "," << chi2_after << std::endl;
        if (chi2_after > boost::math::quantile(chi2_distribution(dof), 0.99)) {
            // std::cout << "rejecting data association" << std::endl;
            while (lm_graph.pose_landmarks.size() > lm_graph.last_landmark_edge) {
                auto *edge = &lm_graph.pose_landmarks.back();
                lm_graph.opt.removeEdge(edge);

                // this seems to fix the segfault problem of online optimization
                // remove any vertices with no edges connected to them
                auto *lm = edge->vertex(1);
                if (lm->edges().size() == 0)
                    lm_graph.opt.removeVertex(lm);
                lm_graph.pose_landmarks.pop_back();
            }
            lm_graph.opt.pop();
            need_reinit = true;
        } else {
            chi2_before = chi2_after;
            lm_graph.opt.discardTop();
            for (auto &lm : lm_graph.landmarks)
                lm.updateEndpoints();
        }
        lm_graph.last_landmark_edge = lm_graph.pose_landmarks.size();
        lm_graph.new_vset.clear();
        lm_graph.new_eset.clear();
    } else if (delta_pose.translation().norm() > 0.01 || std::abs(delta_pose[2]) >= M_PI / 180) {
        // we add points only if the robot moves a little
        // we don't hold lock here, since other threads should never touch the latest pose
        lm_graph.poses.back().addPoints(pc, odom_prop.pose, table);
    }
}

void Drone::addLandmarkObservations(g2o::VertexRhoTheta *landmark, const ls_extractor::LineSegment &bl_line, g2o::VertexSE2 *new_vertex) {
    lm_graph.pose_landmarks.emplace_back();
    auto *pose_landmark = &lm_graph.pose_landmarks.back();
    pose_landmark->vertices()[0] = new_vertex;
    pose_landmark->vertices()[1] = landmark;
    pose_landmark->information().noalias() = bl_line.cov.cast<double>().inverse();
    pose_landmark->setMeasurement(bl_line.rhotheta.cast<double>());
    pose_landmark->start = bl_line.start_;
    pose_landmark->end = bl_line.end_;
    lm_graph.new_eset.insert(pose_landmark);
    // the landmark vertex might have been removed previously due to data association rejection
    if (!lm_graph.opt.vertex(landmark->id())) {
        lm_graph.new_vset.insert(landmark);
        lm_graph.opt.addVertex(landmark);
    }
    lm_graph.opt.addEdge(pose_landmark);
}

// give a line segment's start point and end point, returns the landmark that best matches this segment
g2o::VertexRhoTheta *Drone::mergeLine(const Eigen::Vector2f &start, const Eigen::Vector2f &end) {
    g2o::VertexRhoTheta *closest = nullptr;
    double error = std::numeric_limits<float>::infinity();
    for (auto &landmark : lm_graph.landmarks) {
        if (traveled_dist - landmark.dist >= landmark_max_dist)
            continue;

        Eigen::Vector2f t_p, t_l;
        Eigen::Vector2f lm_rhotheta = landmark.estimate().cast<float>();
        ls_extractor::calc_endpoints<float>(lm_rhotheta, landmark.start, landmark.end, t_l);
        float e = ls_extractor::ll_distance<float>(lm_rhotheta, start, end, t_p);
        if (e < error && !(t_l[0] > t_p[1] + landmark_max_gap || t_l[1] + landmark_max_gap < t_p[0])) {
            closest = &landmark;
            error = e;
        }
    }
    //  && abs(norm_angle(closest->estimate()[1] - rhotheta[1])) < M_PI / 6
    if (error > landmark_assoc_thresh) {
        // if suspect implicit loop closure, allow a higher error margin
        if (closest && error < 1.0f) {
            if (traveled_dist - closest->dist > 15.0 && traveled_dist - closest->dist < landmark_max_dist) {
                return closest;
            }
        }
        // error above threshold: create a new landmark
        lm_graph.landmarks.emplace_back();
        auto &line = lm_graph.landmarks.back();
        line.dist = traveled_dist;
        line.setId(landmark_id++);
        line.setEstimate(ls_extractor::topolar<float>(start, end).cast<double>());
        line.start = start;
        line.end = end;
        lm_graph.new_vset.insert(&line);
        lm_graph.opt.addVertex(&line);
        return &line;
    } else {
        closest->dist = traveled_dist;
        return closest;
    }
}

void Drone::submap_matching() {
    while (ros::ok()) {
        loop_closer.precompute();
        loop_closer.match();
    }
}
