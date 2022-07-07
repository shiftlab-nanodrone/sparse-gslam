#include <geometry_msgs/TransformStamped.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <nav_msgs/Odometry.h>
#include <ros/callback_queue.h>
#include <std_msgs/Float32.h>
#include <tf/tf.h>
#include <tf2/convert.h>
#include <tf2_ros/transform_broadcaster.h>

#include <iostream>

#include "sparse_gslam/RawData.h"
#include "multicloud2.h"
using sparse_gslam::RawData;

typedef message_filters::sync_policies::ApproximateTime<RawData, RawData> MySyncPolicy;
typedef message_filters::Synchronizer<MySyncPolicy> ATS;
constexpr float NaN = std::numeric_limits<float>().quiet_NaN();
class RawDataConverter {
    ros::CallbackQueue cb_queue;
    ros::AsyncSpinner spinner;

    std::string ns;
    DeltaVector deltas;
    g2o::SE2 last_pose;
    ros::Time last_stamp;

    nav_msgs::Odometry odometry;
    geometry_msgs::TransformStamped odom_pc_tf;

    MulticloudConverter pc_converter;
    ros::Publisher odom_pub, rssi_pub, battery_pub;
    tf2_ros::TransformBroadcaster br;
    message_filters::Subscriber<RawData> sub1, sub2;
    ATS ats;

   public:
    RawDataConverter(const std::string& ns, ros::NodeHandle& nh, const XmlRpc::XmlRpcValue& config)
        : spinner(1, &cb_queue),
          ns(ns),
          last_pose(NaN, NaN, 0.0),
          pc_converter(nh, ns, config),

          odom_pub(nh.advertise<nav_msgs::Odometry>(ns + "/odom", 1)),
          rssi_pub(nh.advertise<std_msgs::Float32>(ns + "/rssi", 1)),
          battery_pub(nh.advertise<std_msgs::Float32>(ns + "/battery_voltage", 1)),

          sub1(nh, ns + "/state_xyzv", 1, ros::TransportHints(), &cb_queue),
          sub2(nh, ns + "/state_ranger_qxyzw", 1, ros::TransportHints(), &cb_queue),
          ats(MySyncPolicy(5), sub1, sub2) {
        odom_pc_tf.header.frame_id = odometry.header.frame_id = ns + "/odom";
        odom_pc_tf.child_frame_id = odometry.child_frame_id = ns + "/base_link";

        ats.registerCallback(&RawDataConverter::process_raw_data, this);
        spinner.start();
        std::cout << ns << std::endl;
    }

    ~RawDataConverter() {
        spinner.stop();
    }

    void process_raw_data(const RawData& state1, const RawData& state2) {
        auto stamp = ros::Time::now();  // note: may need to be changed to use the timestamp on crazyflies
        odometry.header.stamp = stamp;

        auto& position = odometry.pose.pose.position;
        position.x = state1.raw[0];
        position.y = state1.raw[1];
        position.z = state1.raw[2];

        auto& linear = odometry.twist.twist.linear;
        linear.x = state1.raw[3];
        linear.y = state1.raw[4];
        linear.z = state1.raw[5];
        auto& angular = odometry.twist.twist.angular;
        angular.x = state1.raw[6];
        angular.y = state1.raw[7];
        angular.z = state1.raw[8];

        auto& rotation = odometry.pose.pose.orientation;
        rotation.x = state2.raw[5];
        rotation.y = state2.raw[6];
        rotation.z = state2.raw[7];
        rotation.w = state2.raw[8];

        odom_pub.publish(odometry);

        std_msgs::Float32 temp;
        temp.data = state1.raw[9];
        battery_pub.publish(temp);

        // maybe add median filter?
        temp.data = state1.raw[10];
        rssi_pub.publish(temp);

        tf::Transform current_scan_tf;
        tf::poseMsgToTF(odometry.pose.pose, current_scan_tf);

        double _r, _p, _y;
        current_scan_tf.getBasis().getRPY(_r, _p, _y);

        if (std::isnan(last_pose[0])) {  // first pose ever?
            last_pose = g2o::SE2(position.x, position.y, _y);
            odom_pc_tf.header.stamp = stamp;
            odom_pc_tf.transform.translation.x = position.x;
            odom_pc_tf.transform.translation.y = position.y;
            odom_pc_tf.transform.translation.z = position.z;
            odom_pc_tf.transform.rotation = rotation;

            br.sendTransform(odom_pc_tf);
        } else {
            g2o::SE2 cur_pose(position.x, position.y, _y);
            deltas.push_back({stamp.toSec() - last_stamp.toSec(), last_pose.inverse() * cur_pose});
            last_pose = cur_pose;
        }
        last_stamp = stamp;
        pc_converter.update(stamp, deltas, current_scan_tf, state2.raw.data());
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "multiscan_converter");
    ros::NodeHandle nh;

    XmlRpc::XmlRpcValue value;
    nh.getParam("swarm/crazyflies", value);

    std::vector<RawDataConverter*> dv;
    for (const auto& p : value)
        dv.push_back(new RawDataConverter(p.first, nh, p.second));

    ros::waitForShutdown();

    for (auto ptr : dv)
        delete ptr;
}