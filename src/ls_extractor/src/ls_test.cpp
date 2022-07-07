#include <chrono>
#include <random>

#include "ls_extractor/impl/smc.h"
#include "ls_extractor/matplotlibcpp.h"

namespace plt = matplotlibcpp;

struct Line {
    double t1, t2;
    Eigen::Vector2d start, dir;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    inline double pl_distance(const Eigen::Vector2d& point, double& t) const {
        auto dvec = point - start;
        t = dvec.dot(dir);
        return (dvec - t * dir).norm();
    }

    void normalize() {
        dir /= dir.norm();
    }

    void plot(const std::string& key) const {
        Eigen::Vector2d start_p = start + t1 * dir;
        Eigen::Vector2d end_p = start + t2 * dir;
        // cout << start_p << "|" << end_p << endl;
        plt::plot({start_p[0], end_p[0]}, {start_p[1], end_p[1]}, key);
    }

    double length() const {
        return t2 - t1;
    }

    Eigen::Vector2d midpoint() const {
        return ((t1 + t2) * 0.5) * dir + start;
    }

    void recalc_start() {
        double t_start = start.dot(dir);
        t1 += t_start;
        t2 += t_start;
        start -= t_start * dir;
    }
};

using LineVector = std::vector<Line, Eigen::aligned_allocator<Line>>;
int main() {
    // plt::ion()
    
    plt::figure_size(720, 720);
    // plt::figure();
    ls_extractor::LineSegmentExtractor extractor;
    LineVector lines;

    std::default_random_engine gen(std::chrono::system_clock::now().time_since_epoch().count());
    std::normal_distribution<float> dist(0.0, 0.03);
    std::normal_distribution<float> dist2(0.0, 0.05);

    lines.push_back({0.0, 2.0, {-1.5, -1.5}, {0.0, 1.0}});
    lines.push_back({0.0, 2.5, {0.5, -1.5}, {0.1, 0.9}});
    lines.push_back({0.0, 1.0, {0.5, -1.5}, {0.9, -0.2}});
    lines.push_back({0.0, 3.0, {-1.0, 1.0}, {0.9, 0.2}});
    // lines.push_back({0.0, 3.0, {-1.0, 1.0}, {0.5, -0.5}});
    for (auto& line : lines) {
        line.normalize();
        line.plot("r-");
    }

    Eigen::Matrix2f cov = (0.01 * 0.01) * Eigen::Matrix2f::Identity();
    std::vector<float> px, py;
    ls_extractor::PointVector pv;
    for (auto& line : lines) {
        for (double t = line.t1; t < line.t2; t += 0.05) {
            Eigen::Vector2f point = (line.start + t * line.dir).cast<float>();
            Eigen::Vector2f normal(-line.dir[1], line.dir[0]);
            point += normal * dist(gen);
            pv.push_back({point, {point.norm(), atan2(point[1], point[0])}, cov});
            px.push_back(point[0]);
            py.push_back(point[1]);
        }
    }

    std::shuffle(pv.begin(), pv.end(), gen);
    extractor.extract_lines(pv);
    
    plt::scatter(px, py, 4.0);
    px.clear();
    py.clear();
    for (auto& seg : extractor.segments) {
        plt::plot<float, float>({seg.start_[0], seg.end_[0]}, {seg.start_[1], seg.end_[1]}, "b-");
    }

    plt::show();
    return 0;
}