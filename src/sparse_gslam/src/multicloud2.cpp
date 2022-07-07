#include "multicloud2.h"

#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>

inline float thresh(float v, float thresh) {
    return v >= thresh ? std::numeric_limits<float>().infinity() : v;
}

MulticloudConverter::MulticloudConverter(ros::NodeHandle& nh,
                                         const XmlRpc::XmlRpcValue& config) : scan_size(config["scan_size"]),
                                                                              mc_window_size(config["multicloud_size"]),
                                                                              var_r(std::pow(config["std_r"], 2.0)),
                                                                              scan_pub(nh.advertise<sensor_msgs::LaserScan>("scan", 2)),
                                                                              mc_pub(nh.advertise<sensor_msgs::PointCloud2>("multicloud", 2)),
                                                                              odom_prop((double)config["std_x"], (double)config["std_y"], (double)config["std_w"]),
                                                                              table(scan_size) {
    temp_bl_cloud.header.frame_id = multicloud.header.frame_id 
        = scan.header.frame_id = "base_link";
    scan.angle_min = (double)config["angle_min"];
    scan.angle_max = (double)config["angle_max"];
    scan.range_min = (double)config["range_min"];
    scan.range_max = (double)config["range_max"];
    scan.ranges.resize(scan_size);
    scan.angle_increment = (scan.angle_max - scan.angle_min) / (scan_size - 1);

    temp_cloud.resize(scan_size);
    temp_bl_cloud.resize(mc_window_size);
    multicloud.reserve(mc_window_size + scan_size * 2);

    pcv.reserve(mc_window_size);
    Jl(0, 0) = Jl(1, 1) = 1.0f;
}

bool MulticloudConverter::update(const ros::Time& stamp, const DeltaVector& deltas, const tf::Transform& current_scan_tf) {
    scan.header.stamp = stamp;
    scan_pub.publish(scan);

    for (int i = 0; i < scan_size; i++) {
        temp_cloud[i].x = table[i].cos_v * thresh(scan.ranges[i], scan.range_max);
        temp_cloud[i].y = table[i].sin_v * thresh(scan.ranges[i], scan.range_max);
        temp_cloud[i].z = 0.0;
    }
    // transform to odom frame
    pcl_ros::transformPointCloud(temp_cloud, temp_cloud, current_scan_tf);  // rotation then translation
    multicloud.insert(multicloud.end(), temp_cloud.begin(), temp_cloud.end());

    if (multicloud.size() >= mc_window_size) {
        memmove(multicloud.points.data(), multicloud.points.data() + multicloud.size() - mc_window_size, mc_window_size * sizeof(pcl::PointXYZ));
        multicloud.points.resize(mc_window_size);
        pcl_ros::transformPointCloud(multicloud, temp_bl_cloud, current_scan_tf.inverse());

        pcv.clear();
        int delta_offset = mc_window_size / scan_size - 1;
        for (int i = 0; i < mc_window_size / scan_size; i++) {
            odom_prop.reset();
            for (int j = i; j < delta_offset; j++) {  // FIXME: boundary
                int k = deltas.size() + j - delta_offset;
                odom_prop.step(deltas[k]);
            }
            float ct = cos((float)odom_prop.pose[2]), st = sin((float)odom_prop.pose[2]);
            Eigen::Matrix3f Juk;  // jacobian of the inverse
            Juk << -ct, st, odom_prop.pose[1] * ct + odom_prop.pose[0] * st,
                -st, -ct, odom_prop.pose[1] * st - odom_prop.pose[0] * ct,
                0.0, 0.0, -1.0;
            odom_prop.cov = Juk * odom_prop.cov * Juk.transpose();
            odom_prop.pose = odom_prop.pose.inverse();
            updateJacobian(Jl, (float)odom_prop.pose[0], (float)odom_prop.pose[1], (float)odom_prop.pose[2]);
            for (int j = 0; j < scan_size; j++) {
                int base = scan_size * i;
                if (std::isfinite(temp_bl_cloud[base + j].x) && std::isfinite(temp_bl_cloud[base + j].y)) {
                    pcv.emplace_back();
                    auto& pt = pcv.back();
                    pt.point[0] = temp_bl_cloud[base + j].x;
                    pt.point[1] = temp_bl_cloud[base + j].y;
                    pt.rhotheta[0] = pt.point.norm();
                    pt.rhotheta[1] = std::atan2(pt.point[1], pt.point[0]);
                    Eigen::Matrix2f covp;  // covariance of the point
                    const float c = table[j].cos_v * table[j].sin_v;
                    covp << table[j].cos_v * table[j].cos_v, c, c, table[j].sin_v * table[j].sin_v;
                    covp *= var_r;
                    pt.cov.noalias() = Jl.block<2, 3>(0, 0) * odom_prop.cov * Jl.block<2, 3>(0, 0).transpose() +
                                       Jl.block<2, 2>(0, 3) * covp * Jl.block<2, 2>(0, 3).transpose();
                }
            }
        }
        // make pcl happy
        temp_bl_cloud.width = mc_window_size;
        temp_bl_cloud.height = 1;
        pcl::toROSMsg(temp_bl_cloud, mc_msg);
        mc_msg.header.stamp = stamp;
        mc_pub.publish(mc_msg);
        return true;
    }
    return false;
}