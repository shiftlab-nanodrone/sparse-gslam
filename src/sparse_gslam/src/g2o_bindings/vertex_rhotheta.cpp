#include "g2o_bindings/vertex_rhotheta.h"
#include "g2o_bindings/edge_se2_rhotheta.h"
#include "g2o/types/slam2d/vertex_se2.h"
#include "ls_extractor/utils.h"
#include "g2o/core/factory.h"
#include "g2o/stuff/macros.h"
namespace g2o {

void VertexRhoTheta::updateEndpoints() {
    if (_edges.size() == 0)
        return;
    Eigen::Vector2d start, dir;
    ls_extractor::calc_start_dir<double>(_estimate, start, dir);
    Eigen::Vector2d t{1e10, -1e10};
    // ls_extractor::calc_endpoints<double>(start, dir, this->start.cast<double>(), this->end.cast<double>(), t);
    for (auto* e : _edges) {
        auto* edge = static_cast<g2o::EdgeSE2RhoTheta*>(e);
        auto* pose = static_cast<g2o::VertexSE2*>(edge->vertex(0));
        Eigen::Vector2d tout;
        ls_extractor::calc_endpoints<double>(start, dir, pose->estimate() * edge->start.cast<double>(), pose->estimate() * edge->end.cast<double>(), tout);
        t[0] = std::min(t[0], tout[0]);
        t[1] = std::max(t[1], tout[1]);
    }
    this->start = (start + t[0] * dir).cast<float>();
    this->end = (start + t[1] * dir).cast<float>();
}

void VertexRhoTheta::setToOriginImpl() { _estimate.setZero(); }

void VertexRhoTheta::oplusImpl(const double* update) {
    _estimate[0] += update[0];
    _estimate[1] += update[1];
    normalize_theta(_estimate[1]);
}

bool VertexRhoTheta::read(std::istream& is) {
    return true;
}

bool VertexRhoTheta::write(std::ostream& os) const {
    return os.good();
}
G2O_REGISTER_TYPE(VERTEX_RHOTHETA, VertexRhoTheta);
}  // namespace g2o