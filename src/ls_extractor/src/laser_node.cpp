#include "ls_extractor/impl/smc.h"
#include "ls_extractor/ros_utils.h"
#include "ls_extractor/visualizer.h"
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

using namespace ls_extractor;

LineSegmentExtractor extractor;
PointVector pcv;

void callback(SegmentVisualizer& vis, sensor_msgs::LaserScanConstPtr _scan) {
    pcv.clear();
    const auto& scan = *_scan;
    float angle = scan.angle_min;
    for (int i = 0; i < scan.ranges.size(); i++, angle += scan.angle_increment) {
        float range = scan.ranges[i];
        if (range >= scan.range_min && range <= scan.range_max) {
            pcv.emplace_back();
            auto& pt = pcv.back();
            pt.point[0] = cos(angle) * range;
            pt.point[1] = sin(angle) * range;
            pt.rhotheta[0] = range;
            pt.rhotheta[1] = angle;
            pt.cov = Eigen::Matrix2f::Identity() * 0.01;
        }
    }
    extractor.extract_lines(pcv);
    vis.visualize(extractor.segments, scan.header.stamp);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "ls_extractor_laser_node");
    ros::NodeHandle nh;
    XmlRpc::XmlRpcValue config;
    std::string frame_id;
    ros::param::get("~config", config);
    ros::param::get("~frame_id", frame_id);
    SegmentVisualizer vis(frame_id, nh);
    auto sub = nh.subscribe<sensor_msgs::LaserScan>("scan", 1, boost::bind(callback, vis, _1));
    ros::spin();
    return 0;
}