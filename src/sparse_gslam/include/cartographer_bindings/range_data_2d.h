#pragma once
#include <ros/message_forward.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <vector>
namespace sensor_msgs {
    ROS_DECLARE_MESSAGE(LaserScan)
}
namespace cartographer {
namespace sensor {
// multi-range data 2d
class RangeData2D {
   public:
    struct MetaData {
        int return_end, end;
        Eigen::Vector2f origin;
        inline void shift_idx(int idx) {
            return_end += idx;
            end += idx;
        }
    };
    std::vector<Eigen::Vector2f> points_;
    std::vector<MetaData> meta_data_;

    void insert_data(const sensor_msgs::LaserScan& scan, const std::vector<Eigen::Vector2f>& table, const Eigen::Vector2f& trans, const Eigen::Rotation2Df& rot);
    void transform_to(const Eigen::Vector2f& trans, const Eigen::Rotation2Df& rot, RangeData2D& out) const;
    void get_returns_pointcloud(std::vector<Eigen::Vector2f>& out) const;
    void clear();
};

}  // namespace sensor
}  // namespace cartographer