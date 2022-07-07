#pragma once
#include "ls_extractor/impl/smc.h"
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

namespace ls_extractor {

class SegmentVisualizer {
   public:
    ros::Publisher seg_marker_pub, split_marker_pub;
    visualization_msgs::Marker seg_markers, split_markers;
    SegmentVisualizer(const string& frame_id, ros::NodeHandle& nh);
    void visualize(const SegmentVector& segments, const ros::Time& stamp);
};

class PCVisualizer {
    std::string frame_id;
    ros::Publisher pub;
    visualization_msgs::MarkerArray markers;
    PCVisualizer(const string& frame_id, ros::NodeHandle& nh);
    void visualize(const PointVector& pcv, const ros::Time& stamp, double scale);
};

}

