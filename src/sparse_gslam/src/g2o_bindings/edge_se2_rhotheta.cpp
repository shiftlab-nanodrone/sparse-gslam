#include "g2o_bindings/edge_se2_rhotheta.h"

#include "g2o/core/factory.h"
#include "g2o/stuff/macros.h"
#include "ls_extractor/utils.h"

namespace g2o {

void EdgeSE2RhoTheta::computeError() {
    const auto* pose = static_cast<VertexSE2*>(_vertices[0]);
    const auto* line = static_cast<VertexRhoTheta*>(_vertices[1]);

    auto poseInv = pose->estimate().inverse();
    _error.noalias() = _measurement - ls_extractor::transform_line<double>(line->estimate(), poseInv.translation(), poseInv.rotation().angle());
    _error[1] = normalize_theta(_error[1]);
}

bool EdgeSE2RhoTheta::read(std::istream& is) {
    return true;
}
bool EdgeSE2RhoTheta::write(std::ostream& os) const {
    return true;
}
G2O_REGISTER_TYPE(EDGE_SE2_RHOTHETA, EdgeSE2RhoTheta);
}  // namespace g2o