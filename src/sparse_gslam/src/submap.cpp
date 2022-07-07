#include "submap.h"

#include <tf/tf.h>

#include "g2o/types/slam2d/vertex_se2.h"
#include "g2o_bindings/vertex_rhotheta.h"

mapping::ValueConversionTables Submap::conversion_tables;

Submap::Submap(int idx, float res) : submap_idx(idx),
                                     probability_grid(mapping::MapLimits(res, Eigen::Vector2d(4., 4.), mapping::CellLimits(160, 160)), &conversion_tables),
                                     high_res_grid(mapping::MapLimits(0.05f, Eigen::Vector2d(4., 4.), mapping::CellLimits(160, 160)), &conversion_tables) {
    grid.header.frame_id = "map";
    grid.info.resolution = res;
}

struct Cell {
    Eigen::Vector2f disp;
    float prob;
};

Eigen::AlignedBox2f Submap::probability_grid_to_occupancy_grid(const g2o::SE2& trans, const mapping::ProbabilityGrid& probability_grid, nav_msgs::OccupancyGrid& grid, float padding) {
    Eigen::AlignedBox2f box;
    std::vector<Cell> cell_array;
    for (const Eigen::Array2i& xy_index : mapping::XYIndexRangeIterator(probability_grid.limits().cell_limits())) {
        if (probability_grid.IsKnown(xy_index)) {
            Eigen::Vector2f disp = probability_grid.limits().GetCellCenter(xy_index);
            box.extend(disp);
            cell_array.push_back({disp, probability_grid.GetProbability(xy_index)});
        }
    }
    float scale = 1.0 / grid.info.resolution;
    box.min() -= Eigen::Vector2f::Constant(padding);
    box.max() += Eigen::Vector2f::Constant(padding);
    Eigen::Vector2d origin = trans * box.min().cast<double>();
    Eigen::Vector2i sz = ((box.max() - box.min()) * scale).array().floor().cast<int>();
    tf::Matrix3x3 orit;
    orit.setRPY(0.0, 0.0, trans[2]);
    tf::Quaternion q;
    orit.getRotation(q);

    grid.info.width = sz[0];
    grid.info.height = sz[1];
    grid.info.origin.position.x = origin[0];
    grid.info.origin.position.y = origin[1];
    tf::quaternionTFToMsg(q, grid.info.origin.orientation);

    grid.data.resize(grid.info.width * grid.info.height);
    memset(grid.data.data(), 127, grid.data.size());
    auto* ptr = reinterpret_cast<unsigned char*>(grid.data.data());
    for (auto& cell : cell_array) {
        Eigen::Vector2i pt = ((cell.disp - box.min()) * scale).array().round().cast<int>();
        ptr[pt[1] * grid.info.width + pt[0]] = 255.0f - cell.prob * 255.0f;
    }
    return box;
}

void Submap::probability_grid_to_occupancy_grid_fixed_size(const mapping::ProbabilityGrid& probability_grid, nav_msgs::OccupancyGrid& grid) {
    Eigen::Vector2f box_min(grid.info.origin.position.x, grid.info.origin.position.y);

    grid.data.resize(grid.info.width * grid.info.height);
    memset(grid.data.data(), 127, grid.data.size());
    auto* ptr = reinterpret_cast<unsigned char*>(grid.data.data());
    const float scale = 1.0 / grid.info.resolution;
    for (const Eigen::Array2i& xy_index : mapping::XYIndexRangeIterator(probability_grid.limits().cell_limits())) {
        if (probability_grid.IsKnown(xy_index)) {
            Eigen::Vector2i pt = ((probability_grid.limits().GetCellCenter(xy_index) - box_min) * scale).array().cast<int>();
            ptr[pt[1] * grid.info.width + pt[0]] = 255.0f - probability_grid.GetProbability(xy_index) * 255.0f;
        }
    }
}

void Submap::fix_submap(g2o::VertexSE2* pose, const mapping::scan_matching::proto::FastCorrelativeScanMatcherOptions2D& options, float padding) {
    this->pose = pose;
    box = probability_grid_to_occupancy_grid(pose->estimate(), probability_grid, grid, padding);
    matcher.emplace(static_cast<const mapping::Grid2D&>(probability_grid), options);
}

void Submap::publish_without_compute(ros::Publisher& pub) {
    if (!pub.getNumSubscribers())
        return;
    Eigen::Vector2d origin = pose->estimate().translation() + Eigen::Rotation2Dd(pose->estimate()[2]) * box.min().cast<double>();
    tf::Matrix3x3 orit;
    orit.setRPY(0.0, 0.0, pose->estimate()[2]);
    tf::Quaternion q;
    orit.getRotation(q);

    grid.info.origin.position.x = origin[0];
    grid.info.origin.position.y = origin[1];
    tf::quaternionTFToMsg(q, grid.info.origin.orientation);
    pub.publish(grid);
}
