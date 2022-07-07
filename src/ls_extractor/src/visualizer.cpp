#include "ls_extractor/visualizer.h"

#include <tf/tf.h>

using namespace std;
namespace ls_extractor {
SegmentVisualizer::SegmentVisualizer(
    const string& frame_id,
    ros::NodeHandle& nh) : seg_marker_pub(nh.advertise<visualization_msgs::Marker>("segment_markers", 2)) {
    seg_markers.header.frame_id = frame_id;
    seg_markers.ns = frame_id + "/segment_markers";
    seg_markers.id = 1;
    seg_markers.type = visualization_msgs::Marker::LINE_LIST;
    seg_markers.scale.x = 0.075;
    seg_markers.color.r = 1.0;
    seg_markers.color.g = 0.0;
    seg_markers.color.b = 0.0;
    seg_markers.color.a = 1.0;
    seg_markers.pose.orientation.w = 1.0;
}

void SegmentVisualizer::visualize(const SegmentVector& segments, const ros::Time& stamp) {
    split_markers.header.stamp = seg_markers.header.stamp = stamp;
    // markers.id++;
    seg_markers.points.resize(segments.size() * 2);
    for (int i = 0; i < segments.size(); i++) {
        seg_markers.points[2 * i].x = segments[i].start_[0];
        seg_markers.points[2 * i].y = segments[i].start_[1];
        seg_markers.points[2 * i + 1].x = segments[i].end_[0];
        seg_markers.points[2 * i + 1].y = segments[i].end_[1];
    }
    seg_marker_pub.publish(seg_markers);
}

PCVisualizer::PCVisualizer(const string& frame_id, ros::NodeHandle& nh) : frame_id(frame_id),
                                                  pub(nh.advertise<visualization_msgs::MarkerArray>("pcv", 2)) {
}

void PCVisualizer::visualize(const PointVector& pcv, const ros::Time& stamp, double scale) {
    markers.markers.resize(pcv.size());
    for (int i = 0; i < pcv.size(); i++) {
        auto& pt = pcv[i];
        auto& marker = markers.markers[i];
        marker.header.frame_id = frame_id;
        marker.header.stamp = stamp;
        marker.ns = frame_id + "/pcv";
        marker.id = i;
        marker.type = visualization_msgs::Marker::CYLINDER;

        const auto& cov = pt.cov;
        float apc2 = 0.5 * (cov(0, 0) + cov(1, 1));  // (a+c)/2
        float amc2 = 0.5 * (cov(0, 0) - cov(1, 1));  // (a-c)/2
        float temp = sqrt(amc2 * amc2 + cov(0, 1) * cov(0, 1));
        float l1 = apc2 + temp, l2 = apc2 - temp;  // 2 eigenvalues
        marker.scale.x = sqrt(l1) * scale;
        marker.scale.y = sqrt(l2) * scale;

        float theta;
        if (abs(cov(0, 1)) < 1e-8) {
            theta = cov(0, 0) >= cov(1, 1) ? 0.0 : M_PI / 2;
        } else {
            theta = atan2(l1 - cov(0, 0), cov(0, 1));
        }

        tf::Matrix3x3 mat;
        tf::Quaternion q;
        mat.setRPY(0.0, 0.0, theta);
        mat.getRotation(q);
        tf::quaternionTFToMsg(q, marker.pose.orientation);

        marker.scale.z = 0.1;
        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 1.0;
        marker.color.a = 1.0;

        marker.pose.position.x = pt.point[0];
        marker.pose.position.y = pt.point[1];
    }
    pub.publish(markers);
}
}  // namespace ls_extractor