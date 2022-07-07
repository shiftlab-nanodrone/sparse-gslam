#pragma once
#include <Eigen/Core>
#include <boost/array.hpp>

namespace ls_extractor {
inline void eigenToCov2d(const Eigen::Matrix2f& cov, boost::array<float, 4UL>& lcov) {
    lcov[0] = cov(0, 0);
    lcov[1] = cov(0, 1);
    lcov[2] = cov(1, 0);
    lcov[3] = cov(1, 1);
}

inline void cov2dToEigen(const boost::array<float, 4UL>& lcov, Eigen::Matrix2f& cov) {
    cov(0, 0) = lcov[0];
    cov(0, 1) = lcov[1];
    cov(1, 0) = lcov[2];
    cov(1, 1) = lcov[3];
}
template <typename T>
using Vector2 = Eigen::Matrix<T, 2, 1>;

template <typename T>
inline void checkRhoTheta(Eigen::Ref<Vector2<T>> rhotheta) {
    if (rhotheta[0] < 0.0) {
        rhotheta[0] = -rhotheta[0];
        rhotheta[1] += M_PI;
        if (rhotheta[1] > M_PI)
            rhotheta[1] -= 2 * M_PI;
    }
}

template <typename T>
Vector2<T> transform_line(const Vector2<T>& rhotheta, const Vector2<T>& trans, T angle) {
    Vector2<T> result = rhotheta;
    result[1] += angle;
    if (result[1] > M_PI)
        result[1] -= 2 * M_PI;
    if (result[1] < -M_PI)
        result[1] += 2 * M_PI;

    Vector2<T> normal(cos(result[1]), sin(result[1]));
    result[0] += trans.dot(normal);
    checkRhoTheta<T>(result);
    return result;
}

template <typename T>
inline Vector2<T> topolar(const Eigen::Ref<const Vector2<T>> start, const Eigen::Ref<const Vector2<T>> end) {
    Vector2<T> result, d = start - end;
    result[1] = atan2(-d[0], d[1]);
    result[0] = start[0] * cos(result[1]) + start[1] * sin(result[1]);
    checkRhoTheta<T>(result);
    return result;
}

template <typename T>
inline void calc_start_dir(const Eigen::Ref<const Vector2<T>> rhotheta, Vector2<T>& start, Vector2<T>& dir) {
    Vector2<T> cos_sin(cos(rhotheta[1]), sin(rhotheta[1]));
    dir = {-cos_sin[1], cos_sin[0]};
    start = rhotheta[0] * cos_sin;
}

template <typename T>
T ll_distance(const Eigen::Ref<const Vector2<T>> rhotheta,
              const Eigen::Ref<const Vector2<T>> p1,
              const Eigen::Ref<const Vector2<T>> p2,
              Eigen::Ref<Vector2<T>> tout) {
    Vector2<T> start, dir;
    calc_start_dir<T>(rhotheta, start, dir);

    Vector2<T> dvec1 = p1 - start;
    tout[0] = dvec1.dot(dir);
    Vector2<T> dvec2 = p2 - start;
    tout[1] = dvec2.dot(dir);

    T error = (dvec1 - tout[0] * dir).norm() + (dvec2 - tout[1] * dir).norm();
    if (tout[1] < tout[0])
        std::swap(tout[0], tout[1]);
    return error;
}

template <typename T>
inline void calc_endpoints(const Eigen::Ref<const Vector2<T>> start,
                           const Eigen::Ref<const Vector2<T>> dir,
                           const Eigen::Ref<const Vector2<T>> p1,
                           const Eigen::Ref<const Vector2<T>> p2,
                           Eigen::Ref<Vector2<T>> tout) {
    tout[0] = (p1 - start).dot(dir);
    tout[1] = (p2 - start).dot(dir);
    if (tout[1] < tout[0])
        std::swap(tout[0], tout[1]);
}

template <typename T>
inline void calc_endpoints(const Eigen::Ref<const Vector2<T>> rhotheta,
                           const Eigen::Ref<const Vector2<T>> p1,
                           const Eigen::Ref<const Vector2<T>> p2,
                           Eigen::Ref<Vector2<T>> tout) {
    Vector2<T> start, dir;
    calc_start_dir<T>(rhotheta, start, dir);
    calc_endpoints<T>(start, dir, p1, p2, tout);
}
}  // namespace ls_extractor