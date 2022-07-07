#include "cartographer_bindings/range_data_2d.h"

#include <sensor_msgs/LaserScan.h>

namespace cartographer {
namespace sensor {

void RangeData2D::insert_data(const sensor_msgs::LaserScan& scan, const std::vector<Eigen::Vector2f>& table, const Eigen::Vector2f& trans, const Eigen::Rotation2Df& rot) {
    meta_data_.emplace_back();
    auto& meta = meta_data_.back();
    meta.origin = trans;
    int scan_size = scan.ranges.size();
    for (int i = 0; i < scan_size; i++) {
        float dist = scan.ranges[i];
        if (dist < scan.range_max) {
            points_.push_back(trans + rot * (table[i] * dist));
        }
    }
    meta.return_end = points_.size();
    for (int i = 0; i < scan_size; i++) {
        float dist = scan.ranges[i];
        if (!std::isnan(dist) && dist >= scan.range_max) {
            points_.push_back(trans + rot * (table[i] * scan.range_max));
        }
    }
    meta.end = points_.size();
}

void RangeData2D::transform_to(const Eigen::Vector2f& trans, const Eigen::Rotation2Df& rot, RangeData2D& out) const {
    int p_start_idx = out.points_.size();
    int m_start_idx = out.meta_data_.size();
    out.points_.resize(p_start_idx + points_.size());
    out.meta_data_.resize(m_start_idx + meta_data_.size());

    std::transform(points_.begin(), points_.end(), out.points_.begin() + p_start_idx, [&rot, &trans](const auto& pt) { return (rot * pt + trans).eval(); });
    std::transform(meta_data_.begin(), meta_data_.end(), out.meta_data_.begin() + m_start_idx, [&trans, p_start_idx](MetaData meta) {
        meta.shift_idx(p_start_idx);
        meta.origin += trans;
        return meta;
    });
}

void RangeData2D::get_returns_pointcloud(std::vector<Eigen::Vector2f>& out) const {
    out.reserve(points_.size());
    int i = 0;
    for (const auto& meta : meta_data_) {
        for (; i < meta.return_end; i++) {
            out.push_back(points_[i]);
        }
        i = meta.end;
    }
}

void RangeData2D::clear() {
    points_.clear();
    meta_data_.clear();
}
}  // namespace sensor
}  // namespace cartographer