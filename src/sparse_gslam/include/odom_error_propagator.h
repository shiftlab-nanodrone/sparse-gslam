#pragma once
#include <Eigen/Core>

#include "delta_vector.h"

template <typename T, int r, int c>
inline void updateJacobian(Eigen::Matrix<T, r, c>& J, T dx, T dy, T theta) {
    T ct = cos(theta), st = sin(theta);
    J(0, 2) = dy * ct - dx * st;
    J(0, 3) = ct;
    J(0, 4) = st;
    J(1, 2) = -dx * ct - dy * st;
    J(1, 3) = -st;
    J(1, 4) = ct;
}

template <typename T>
class OdomErrorPropagator {
   public:
    g2o::SE2 pose;
    const T var_x, var_y, var_w;
    Eigen::Matrix<T, 3, 3> cov;  // x, y, theta
    OdomErrorPropagator(T std_x, T std_y, T std_w, const g2o::SE2& pose = g2o::SE2()) : pose(pose),
                                                                                        var_x(std_x * std_x),
                                                                                        var_y(std_y * std_y),
                                                                                        var_w(std_w * std_w),
                                                                                        cov(Eigen::Matrix<T, 3, 3>::Identity() * 1e-6) {
        J << 1.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 1.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 1.0, 0.0, 0.0, 1.0;
    }

    void reset() {
        cov = Eigen::Matrix<T, 3, 3>::Identity() * 1e-6;
        pose = g2o::SE2();
    }

    void step(const Delta& delta) {
        updateJacobian(J, (T)delta.dpose[0], (T)delta.dpose[1], (T)pose[2]);
        covu.diagonal() << std::abs(delta.dpose[0] * delta.dpose[0]) * var_x,
            std::abs(delta.dpose[1] * delta.dpose[0]) * var_y,
            std::abs(delta.dpose[2] * delta.dpose[0]) * var_w;
        cov.template block<3, 3>(0, 0) = J.template block<3, 3>(0, 0) * cov * J.template block<3, 3>(0, 0).transpose() +
                                         J.template block<3, 3>(0, 3) * covu * J.template block<3, 3>(0, 3).transpose();
        pose *= delta.dpose;
    }

   private:
    Eigen::DiagonalMatrix<T, 3> covu;  // vx, vy, vtheta
    Eigen::Matrix<T, 3, 6> J;
};
