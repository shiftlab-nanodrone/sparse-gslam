#pragma once

#include <memory>
namespace ros {
    class NodeHandle;
    class Time;
}
namespace XmlRpc {
    class XmlRpcValue;
}
class Drone;

class Visualizer {
   public:
    Visualizer(Drone& drone, ros::NodeHandle& nh, const XmlRpc::XmlRpcValue& config) noexcept;
    ~Visualizer();
    void visualize_landmarks(const ros::Time& stamp);
    void start(int rate);
    void stop();

   private:
    struct Impl;
    std::unique_ptr<Impl> impl;
};