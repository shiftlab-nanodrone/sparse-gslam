#include "pose_with_observation.h"
#include <sensor_msgs/LaserScan.h>
using namespace cartographer;

PoseWithObservation::PoseWithObservation(const sensor_msgs::LaserScan& scan, const std::vector<Eigen::Vector2f>& table, const g2o::SE2& raw_odom) {
    data.insert_data(scan, table, Eigen::Vector2f::Zero(), Eigen::Rotation2Df::Identity());
    odom.push_back({scan.header.stamp.toSec(), raw_odom});
}
void PoseWithObservation::addPoints(const sensor_msgs::LaserScan& scan, const g2o::SE2& transform, const std::vector<Eigen::Vector2f>& table) {
    data.insert_data(scan, table, transform.translation().cast<float>(), transform.rotation().cast<float>());
    odom.push_back({scan.header.stamp.toSec(), transform});
}

void PoseWithObservation::construct_multicloud_with_correct(const PoseWithObservations& poses, int start, int mid, int end, sensor::RangeData2D& result) {
    auto pose_inv = poses[mid].pose.estimate().inverse();
    result.clear();
    for (auto it = poses.begin() + start, eit = poses.begin() + end; it != eit; it++) {
        auto trans = pose_inv * it->pose.estimate();
        it->data.transform_to(trans.translation().cast<float>(), trans.rotation().cast<float>(), result);
    }
}

void PoseWithObservation::construct_multicloud_with_correct_returns_only(const PoseWithObservations& poses, int start, int mid, int end, std::vector<Eigen::Vector2f>& result) {
    auto pose_inv = poses[mid].pose.estimate().inverse();
    result.clear();
    for (auto it = poses.begin() + start, eit = poses.begin() + end; it != eit; it++) {
        auto transf = pose_inv * it->pose.estimate();
        Eigen::Vector2f trans = transf.translation().cast<float>();
        Eigen::Matrix2f rot = transf.rotation().cast<float>().toRotationMatrix();
        int i = 0;
        for (const auto& meta : it->data.meta_data_) {
            for (; i < meta.return_end; i++) {    
                result.push_back(rot * it->data.points_[i] + trans);
            }
            i = meta.end;
        }
    }
}
