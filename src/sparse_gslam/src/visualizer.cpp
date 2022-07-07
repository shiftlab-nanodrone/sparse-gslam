#include "visualizer.h"

#include <geometry_msgs/PoseArray.h>
#include <nav_msgs/OccupancyGrid.h>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

#include <thread>

#include "drone.h"
#include "submap.h"
// #define USE_CARTOGRAPHER_GRID

template <typename T>
inline void eigen_to_point(const Eigen::MatrixBase<T> &vec, geometry_msgs::Point &pt, double z = 0.0) {
    pt.x = vec[0];
    pt.y = vec[1];
    pt.z = z;
}

inline void se2_to_pose(const g2o::SE2 &se2, geometry_msgs::Pose &pose, double z = 0.0) {
    pose.position.x = se2[0];
    pose.position.y = se2[1];
    pose.position.z = z;
    double half = se2[2] / 2;
    pose.orientation.z = sin(half);
    pose.orientation.w = cos(half);
}

template <typename T>
void visualize_closures(const T &closures, visualization_msgs::Marker &marker) {
    marker.points.resize(closures.size() * 2);
    int i = 0;
    for (auto *edge : closures) {
        auto *src = static_cast<g2o::VertexSE2 *>(edge->vertices()[0]);
        auto *dst = static_cast<g2o::VertexSE2 *>(edge->vertices()[1]);
        eigen_to_point(src->estimate().translation(), marker.points[2 * i], 0.0);
        eigen_to_point(dst->estimate().translation(), marker.points[2 * i + 1], 0.0);
        i++;
    }
}

// insert range data directly into occupancy grid
class RangeDataInserter {
    const float missLogOdds;
    const float hitLogOdds;

   public:
    RangeDataInserter(float hitProb, float missProb) : missLogOdds(std::log(missProb / (1 - missProb))), hitLogOdds(std::log(hitProb / (1 - hitProb))) {
    }

    // Bresenham's line algorithm
    // adapted from https://rosettacode.org/wiki/Bitmap/Bresenham%27s_line_algorithm#C.2B.2B
    static void Line(float x1, float y1, float x2, float y2, float val, int width, float *float_grid) {
        const bool steep = (fabs(y2 - y1) > fabs(x2 - x1));
        if (steep) {
            std::swap(x1, y1);
            std::swap(x2, y2);
        }
        if (x1 > x2) {
            std::swap(x1, x2);
            std::swap(y1, y2);
        }

        const float dx = x2 - x1;
        const float dy = fabs(y2 - y1);

        float error = dx / 2.0f;
        const int ystep = (y1 < y2) ? 1 : -1;
        int y = (int)y1;
        const int maxX = (int)x2;
        if (steep) {
            for (int x = (int)x1; x <= maxX; x++) {
                float_grid[x * width + y] += val;
                error -= dy;
                if (error < 0) {
                    y += ystep;
                    error += dx;
                }
            }
        } else {
            for (int x = (int)x1; x <= maxX; x++) {
                float_grid[y * width + x] += val;
                error -= dy;
                if (error < 0) {
                    y += ystep;
                    error += dx;
                }
            }
        }
    }

    void rayTrace(const cartographer::sensor::RangeData2D &range_data, nav_msgs::OccupancyGrid &grid, float padding) {
        Eigen::AlignedBox2f box;
        for (const auto &pt : range_data.points_)
            box.extend(pt);
        for (const auto &meta : range_data.meta_data_)
            box.extend(meta.origin);

        float scale = 1.0 / grid.info.resolution;
        box.min() -= Eigen::Vector2f::Constant(padding);
        box.max() += Eigen::Vector2f::Constant(padding);
        Eigen::Vector2i sz = ((box.max() - box.min()) * scale).array().floor().cast<int>();

        grid.info.width = sz[0];
        grid.info.height = sz[1];
        grid.info.origin.position.x = box.min().x();
        grid.info.origin.position.y = box.min().y();

        grid.data.clear();
        grid.data.resize(sz[0] * sz[1] * sizeof(float), 0);
        float *float_grid = (float *)grid.data.data();
        int i = 0;
        for (const auto &meta : range_data.meta_data_) {
            Eigen::Vector2f transOrg = (meta.origin - box.min()) * scale;
            for (; i < meta.return_end; i++) {
                Eigen::Vector2f transPt = (range_data.points_[i] - box.min()) * scale;
                Eigen::Vector2i transPti = transPt.array().round().cast<int>();

                int idx = transPti[1] * sz[0] + transPti[0];
                float prevVal = float_grid[idx];
                Line(transOrg[0], transOrg[1], transPt[0], transPt[1], missLogOdds, sz[0], float_grid);
                // if hit also traced as miss, subtract the odds of a miss
                float_grid[idx] += hitLogOdds - (float_grid[idx] != prevVal) * missLogOdds;
            }
            for (; i < meta.end; i++) {
                Eigen::Vector2f transPt = (range_data.points_[i] - box.min()) * scale;
                Line(transOrg[0], transOrg[1], transPt[0], transPt[1], missLogOdds, sz[0], float_grid);
            }
        }
        int g_size = sz[0] * sz[1];
        for (i = 0; i < g_size; i++) {
            float exp_odds = exp(float_grid[i]);
            grid.data[i] = 255.0f - 255.0f * exp_odds / (exp_odds + 1);
        }
        grid.data.resize(g_size);
    }
};

constexpr bool fixed_map = false;
inline void convert_prob_grid(
    const Drone &drone,
    mapping::ProbabilityGrid &grid,
    const sensor::RangeData2D &range,
    nav_msgs::OccupancyGrid &grid_msg,
    mapping::ValueConversionTables &table) {
    grid.~ProbabilityGrid();
    new (&grid) mapping::ProbabilityGrid(grid.limits(), &table);

    drone.loop_closer.range_data_inserter.Insert(range, &grid);
    if (fixed_map)
        Submap::probability_grid_to_occupancy_grid_fixed_size(grid, grid_msg);
    else
        Submap::probability_grid_to_occupancy_grid(g2o::SE2(), grid, grid_msg, 5.0f);
}

struct Visualizer::Impl {
    static cartographer::mapping::ValueConversionTables conversion_table;
    Drone &drone;
#ifdef USE_CARTOGRAPHER_GRID
    mapping::ProbabilityGrid o_grid, l_grid, p_grid;
#else
    RangeDataInserter inserter;
#endif
    sensor::RangeData2D o_range, l_range, p_range;
    nav_msgs::OccupancyGrid o_grid_msg, l_grid_msg, p_grid_msg;
    ros::Publisher o_grid_pub, l_grid_pub, p_grid_pub;

    visualization_msgs::Marker landmark_marker, edge_marker;
    ros::Publisher landmark_marker_pub, edge_marker_pub;

    visualization_msgs::Marker closure_marker, false_closure_marker, false_candidate_marker;
    ros::Publisher closure_pub, false_closure_pub, false_candidate_pub;

    geometry_msgs::PoseArray odom_poses, lm_poses, pg_poses;
    ros::Publisher odom_poses_pub, lm_poses_pub, pg_poses_pub;

    visualization_msgs::Marker match_submap_marker;
    ros::Publisher match_submap_pub;

    std::atomic<bool> status;
    std::thread vis_thread;

    Impl(Drone &drone, ros::NodeHandle &nh,
         const XmlRpc::XmlRpcValue &config) : drone(drone),
#ifdef USE_CARTOGRAPHER_GRID                      
                                              o_grid(mapping::MapLimits(config["map_resolution"], Eigen::Vector2d(4., 4.), mapping::CellLimits(160, 160)), &conversion_table),
                                              l_grid(mapping::MapLimits(config["map_resolution"], Eigen::Vector2d(4., 4.), mapping::CellLimits(160, 160)), &conversion_table),
                                              p_grid(mapping::MapLimits(config["map_resolution"], Eigen::Vector2d(4., 4.), mapping::CellLimits(160, 160)), &conversion_table),
#else
                                              inserter((double)config["hit_probability"], (double)config["miss_probability"]),
#endif
                                              o_grid_pub(nh.advertise<nav_msgs::OccupancyGrid>("/odom_map", 5)),
                                              l_grid_pub(nh.advertise<nav_msgs::OccupancyGrid>("/landmark_map", 5)),
                                              p_grid_pub(nh.advertise<nav_msgs::OccupancyGrid>("/pose_map", 5)),
                                              landmark_marker_pub(nh.advertise<visualization_msgs::Marker>("/landmarks", 5)),
                                              edge_marker_pub(nh.advertise<visualization_msgs::Marker>("/graph_edges", 1)),
                                              closure_pub(nh.advertise<visualization_msgs::Marker>("/loop_closures", 1)),
                                              false_closure_pub(nh.advertise<visualization_msgs::Marker>("/false_loop_closures", 1)),
                                              false_candidate_pub(nh.advertise<visualization_msgs::Marker>("/false_candidates", 1)),
                                              odom_poses_pub(nh.advertise<geometry_msgs::PoseArray>("odom_poses", 1)),
                                              lm_poses_pub(nh.advertise<geometry_msgs::PoseArray>("landmark_poses", 1)),
                                              pg_poses_pub(nh.advertise<geometry_msgs::PoseArray>("pose_graph_poses", 1)),
                                              match_submap_pub(nh.advertise<visualization_msgs::Marker>("/match_submap", 1)) {
        landmark_marker.header.frame_id = edge_marker.header.frame_id =
            closure_marker.header.frame_id = false_closure_marker.header.frame_id =
                false_candidate_marker.header.frame_id = match_submap_marker.header.frame_id =
                    odom_poses.header.frame_id = lm_poses.header.frame_id =
                        pg_poses.header.frame_id = "map";

        o_grid_msg.info.resolution = (double)config["map_resolution"];
        o_grid_msg.info.origin.orientation.w = 1.0;

        // for fixed size map
        if (fixed_map) {
            constexpr float SZ = 150.0;
            o_grid_msg.info.width = SZ / o_grid_msg.info.resolution;
            o_grid_msg.info.height = SZ / o_grid_msg.info.resolution;
            o_grid_msg.data.resize(o_grid_msg.info.width * o_grid_msg.info.height);
            o_grid_msg.info.origin.position.x = -SZ / 2;
            o_grid_msg.info.origin.position.y = -SZ / 2;
        }

        o_grid_msg.header.frame_id = "map";
        l_grid_msg = o_grid_msg;
        p_grid_msg = o_grid_msg;

        landmark_marker.ns = "landmarks";
        landmark_marker.id = 1;
        landmark_marker.type = visualization_msgs::Marker::LINE_LIST;
        landmark_marker.scale.x = 0.1;
        landmark_marker.pose.orientation.w = 1.0;
        landmark_marker.color.b = 1.0;
        landmark_marker.color.a = 1.0;

        edge_marker.ns = "edges";
        edge_marker.id = 1;
        edge_marker.type = visualization_msgs::Marker::LINE_LIST;
        edge_marker.scale.x = 0.025;
        edge_marker.pose.orientation.w = 1.0;
        edge_marker.color.b = 0.0;
        edge_marker.color.g = 0.5;
        edge_marker.color.a = 1.0;

        closure_marker.ns = "closure";
        closure_marker.id = 1;
        closure_marker.type = visualization_msgs::Marker::LINE_LIST;
        closure_marker.scale.x = 0.1;
        closure_marker.pose.orientation.w = 1.0;
        closure_marker.color.g = 1.0;
        closure_marker.color.a = 1.0;

        false_closure_marker.ns = "false_closure";
        false_closure_marker.id = 1;
        false_closure_marker.type = visualization_msgs::Marker::LINE_LIST;
        false_closure_marker.scale.x = 0.1;
        false_closure_marker.pose.orientation.w = 1.0;
        false_closure_marker.color.r = 1.0;
        false_closure_marker.color.a = 1.0;

        false_candidate_marker.ns = "false_candidate_marker";
        false_candidate_marker.id = 1;
        false_candidate_marker.type = visualization_msgs::Marker::LINE_LIST;
        false_candidate_marker.scale.x = 0.1;
        false_candidate_marker.pose.orientation.w = 1.0;
        false_candidate_marker.color.b = 1.0;
        false_candidate_marker.color.a = 1.0;

        match_submap_marker.ns = "match_submap";
        match_submap_marker.id = 1;
        match_submap_marker.type = visualization_msgs::Marker::POINTS;
        match_submap_marker.scale.x = 0.4;
        match_submap_marker.scale.y = 0.4;
        match_submap_marker.scale.z = 0.4;
        match_submap_marker.pose.orientation.w = 1.0;
        match_submap_marker.color.b = 1.0;
        match_submap_marker.color.a = 1.0;
    }
    void visualize_landmarks(const ros::Time &stamp) {
        landmark_marker.header.stamp = edge_marker.header.stamp =
            closure_marker.header.stamp = false_closure_marker.header.stamp =
                false_candidate_marker.header.stamp = match_submap_marker.header.stamp =
                    odom_poses.header.stamp = lm_poses.header.stamp =
                        pg_poses.header.stamp = stamp;
        landmark_marker.points.clear();
        edge_marker.points.clear();
        o_range.clear();
        l_range.clear();
        p_range.clear();

        // the critical section is very fast: usually <0.005s time even for large maps like mit-killian
        {
            boost::shared_lock<boost::shared_mutex> lm_lock(drone.lm_graph.mu);
            for (const auto &landmark : drone.lm_graph.landmarks) {
                landmark_marker.points.emplace_back();
                eigen_to_point(landmark.start, landmark_marker.points.back());
                landmark_marker.points.emplace_back();
                eigen_to_point(landmark.end, landmark_marker.points.back());

                Eigen::Vector2f midpoint = (landmark.start + landmark.end) * 0.5f;
                for (auto *edge : landmark.edges()) {
                    auto *pose = static_cast<g2o::VertexSE2 *>(edge->vertices()[0]);
                    edge_marker.points.emplace_back();
                    eigen_to_point(pose->estimate().translation(), edge_marker.points.back());

                    edge_marker.points.emplace_back();
                    eigen_to_point(midpoint, edge_marker.points.back());
                }
            }
            match_submap_marker.points.clear();
            for (const auto &info : drone.loop_closer.submaps_info) {
                match_submap_marker.points.emplace_back();
                match_submap_marker.points.back().x = info.x;
                match_submap_marker.points.back().y = info.y;
            }

            boost::shared_lock<boost::shared_mutex> pose_lock(drone.pose_graph.mu);
            visualize_closures(drone.pose_graph.closures, closure_marker);
            visualize_closures(drone.pose_graph.false_closures, false_closure_marker);
            visualize_closures(drone.pose_graph.false_candidates, false_candidate_marker);

            if (drone.lm_graph.poses.size() >= 2) {
#ifdef SHOW_RAW_ODOM
                odom_poses.poses.resize(drone.lm_graph.poses.size());
                auto it0 = odom_poses.poses.begin();
#endif
                lm_poses.poses.resize(drone.lm_graph.poses.size());
                auto it1 = lm_poses.poses.begin();

                pg_poses.poses.resize(drone.lm_graph.poses.size());
                auto it2 = pg_poses.poses.begin();

                auto lit = drone.lm_graph.poses.begin();
                auto pit = drone.pose_graph.poses.begin();
                auto end = drone.lm_graph.poses.begin() + drone.loop_closer.last_opt_pose_index;
                end -= end == drone.lm_graph.poses.end();  // do not include the last pose
                for (; lit != end; lit++, pit++) {
#ifdef SHOW_RAW_ODOM
                    se2_to_pose(lit->odom[0].dpose, *it0++);
                    lit->data.transform_to(lit->odom[0].dpose.translation().cast<float>(), lit->odom[0].dpose.rotation().cast<float>(), o_range);
#endif
                    se2_to_pose(lit->pose.estimate(), *it1++);
                    lit->data.transform_to(lit->pose.estimate().translation().cast<float>(), lit->pose.estimate().rotation().cast<float>(), l_range);

                    se2_to_pose(pit->pose.estimate(), *it2++);
                    lit->data.transform_to(pit->pose.estimate().translation().cast<float>(), pit->pose.estimate().rotation().cast<float>(), p_range);
                }
                g2o::SE2 estimate = (pit - 1)->pose.estimate();
                end = drone.lm_graph.poses.end() - 1;
                for (; lit != end; lit++) {
#ifdef SHOW_RAW_ODOM
                    se2_to_pose(lit->odom[0].dpose, *it0++);
                    lit->data.transform_to(lit->odom[0].dpose.translation().cast<float>(), lit->odom[0].dpose.rotation().cast<float>(), o_range);
#endif
                    se2_to_pose(lit->pose.estimate(), *it1++);
                    lit->data.transform_to(lit->pose.estimate().translation().cast<float>(), lit->pose.estimate().rotation().cast<float>(), l_range);

                    estimate = estimate * ((lit - 1)->pose.estimate().inverse() * lit->pose.estimate());
                    // estimate = estimate * lit->edge.measurement();
                    se2_to_pose(estimate, *it2++);
                    lit->data.transform_to(estimate.translation().cast<float>(), estimate.rotation().cast<float>(), p_range);
                }
            }
        }

        // this part is quite slow (~0.4s for large maps like mit-killian)
        // but it's ok because no locks are required
        // auto start = ros::Time::now().toSec();

#ifdef USE_CARTOGRAPHER_GRID
        // if we use cartographer, we need to first insert range into cartographer's grid and then convert it to ros occupancy grid
        // which is quite slow
        convert_prob_grid(drone, o_grid, o_range, o_grid_msg, conversion_table);
        convert_prob_grid(drone, l_grid, l_range, l_grid_msg, conversion_table);
        convert_prob_grid(drone, p_grid, p_range, p_grid_msg, conversion_table);
#else
        // this is much faster
#ifdef SHOW_RAW_ODOM
        inserter.rayTrace(o_range, o_grid_msg, 5.0);
#endif
        inserter.rayTrace(l_range, l_grid_msg, 5.0);
        inserter.rayTrace(p_range, p_grid_msg, 5.0);
#endif
        // std::cout << "map conv time: " << ros::Time::now().toSec() - start << std::endl;
#ifdef SHOW_RAW_ODOM
        o_grid_pub.publish(o_grid_msg);
        odom_poses_pub.publish(odom_poses);
#endif
        p_grid_pub.publish(p_grid_msg);
        pg_poses_pub.publish(pg_poses);

        l_grid_pub.publish(l_grid_msg);
        lm_poses_pub.publish(lm_poses);

        landmark_marker_pub.publish(landmark_marker);
        edge_marker_pub.publish(edge_marker);

        closure_pub.publish(closure_marker);
        false_closure_pub.publish(false_closure_marker);
        false_candidate_pub.publish(false_candidate_marker);
        match_submap_pub.publish(match_submap_marker);
    }

    void start(int rate) {
        status = true;
        vis_thread = std::thread(&Impl::visualize_thread, this, rate);
    }

    void stop() {
        status = false;
        vis_thread.join();
    }

    void visualize_thread(int r) {
        ros::Rate rate(r);
        while (status) {
            visualize_landmarks(ros::Time::now());
            rate.sleep();
        }
    }
};

mapping::ValueConversionTables Visualizer::Impl::conversion_table;

Visualizer::Visualizer(Drone &drone, ros::NodeHandle &nh,
                    const XmlRpc::XmlRpcValue &config) noexcept : impl(new Visualizer::Impl(drone, nh, config)) {}

Visualizer::~Visualizer() {}

void Visualizer::visualize_landmarks(const ros::Time &stamp) {
    impl->visualize_landmarks(stamp);
}

void Visualizer::start(int rate) {
    impl->start(rate);
}

void Visualizer::stop() {
    impl->stop();
}