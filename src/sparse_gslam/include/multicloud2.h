#pragma once
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>

#include "ls_extractor/defs.h"
#include "odom_error_propagator.h"
namespace tf {
    class Transform;
}
class MulticloudConverter {
   public:
    const int scan_size, mc_window_size;
    const float var_r;

    ros::Publisher scan_pub, mc_pub, mc_cov_pub;
    sensor_msgs::PointCloud2 mc_msg;
    pcl::PointCloud<pcl::PointXYZ> multicloud, temp_cloud, temp_bl_cloud;
    OdomErrorPropagator<float> odom_prop;
    Eigen::Matrix<float, 2, 5> Jl = Eigen::Matrix<float, 2, 5>::Zero();
    ls_extractor::PointVector pcv;

    sensor_msgs::LaserScan scan;
    struct CosSin {
        float cos_v, sin_v;
    };
    std::vector<CosSin> table;
    MulticloudConverter(ros::NodeHandle& nh, const XmlRpc::XmlRpcValue& config);

    bool update(const ros::Time& stamp, const DeltaVector& deltas, const tf::Transform& current_scan_tf);
};