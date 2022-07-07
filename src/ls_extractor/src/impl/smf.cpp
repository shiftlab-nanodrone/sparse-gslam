#include "ls_extractor/impl/smf.h"

#include <Eigen/Dense>
#include <iostream>

#include "ls_extractor/utils.h"

namespace ls_extractor {
using namespace std;

float pairwise_closest(const vector<const Point*>& points1, const vector<const Point*>& points2) {
    float dist = 1e10;
    for (auto it1 : points1) {
        for (auto it2 : points2) {
            float pd = (it1->point - it2->point).squaredNorm();
            if (pd < dist)
                dist = pd;
        }
    }
    return sqrt(dist);
}

LineSegment::LineSegment(const PointVector& points) : points(points.size()), uj(points.size(), 1.0f), dj(points.size()) {
    for (int i = 0; i < points.size(); i++) {
        this->points[i] = &points[i];
    }
}
LineSegment::LineSegment(const vector<const Point*>& points) : points(points), uj(points.size(), 1.0f), dj(points.size()) {}

float LineSegment::calc_xybar(Eigen::Vector2f& xybar, float m) const {
    xybar = Eigen::Vector2f::Zero();
    float denum = 0.0;
    for (int i = 0; i < points.size(); i++) {
        auto it = points[i];
        float um = pow(uj[i], m);
        xybar += it->point * um;
        denum += um;
    }
    xybar /= denum;
    return denum;
}

void LineSegment::calc_dij(const Eigen::Vector2f& xybar) {
    float ct = cos(rhotheta[1]), st = sin(rhotheta[1]);
    for (int i = 0; i < points.size(); i++) {
        auto it = points[i];
        float dist = rhotheta[0] - it->point[0] * ct - it->point[1] * st;
        dj[i] = dist * dist + (it->point - xybar).squaredNorm();
    }
}

float LineSegment::calc_dispersion() const {
    Eigen::Vector2f trig(cos(rhotheta[1]), sin(rhotheta[1]));
    float disp = 0.0;
    for (auto it : points) {
        float v = rhotheta[0] - it->point.dot(trig);
        disp += v * v;
    }
    disp /= points.size();
    return sqrt(disp);
}

// see Mobile robot SLAM for line-based environment representation (appendix)
// https://folk.ntnu.no/skoge/prost/proceedings/cdc-ecc05/pdffiles/papers/2714.pdf
void LineSegment::leastSqFit(const float m, bool calc_dij_flag, bool calc_cov) {
    Eigen::Vector2f Sx2y2 = Eigen::Vector2f::Zero(), xybar;
    float sum_uj = calc_xybar(xybar, m);
    float Sxy = 0.0;
    // this is weighted least square fit
    for (int i = 0; i < points.size(); i++) {
        auto it = points[i];
        float um = pow(uj[i], m);
        Sx2y2 += um * (it->point - xybar).array().square().matrix();
        Sxy += um * (it->point - xybar).prod();
    }
    float Sy2_Sx2 = Sx2y2[1] - Sx2y2[0];
    rhotheta[1] = 0.5 * atan2(-2 * Sxy, Sy2_Sx2);

    // make sure that rho >= 0
    float ct = cos(rhotheta[1]), st = sin(rhotheta[1]);
    checkRhoTheta<float>(rhotheta);  // make sure that rho >= 0
    if (calc_dij_flag)
        calc_dij(xybar);

    if (!calc_cov)
        return;

    ct = cos(rhotheta[1]), st = sin(rhotheta[1]);
    // compute covariance matrix of the line parameters
    float xbar_st = xybar[0] * st, ybar_ct = xybar[1] * ct;
    cov = Eigen::Matrix2f::Zero();
    float denum = 1.0 / (Sy2_Sx2 * Sy2_Sx2 + 4 * Sxy * Sxy);
    float ct_n = ct / sum_uj, st_n = st / sum_uj;
    for (int i = 0; i < points.size(); i++) {
        auto it = points[i];
        float u = pow(uj[i], m);
        Eigen::Matrix2f Ai;
        Eigen::Vector2f d = xybar - u * it->point;

        // compute the Jacobian for each point
        Ai(1, 0) = (d[1] * Sy2_Sx2 + 2 * Sxy * d[0]) * denum;
        Ai(1, 1) = (d[0] * Sy2_Sx2 - 2 * Sxy * d[1]) * denum;
        Ai(0, 0) = ct_n * u - xbar_st * Ai(1, 0) + ybar_ct * Ai(1, 0);
        Ai(0, 1) = st_n * u - xbar_st * Ai(1, 1) + ybar_ct * Ai(1, 1);
        cov += Ai * it->cov * Ai.transpose();
    }
}
void LineSegment::projectEndpoints() {
    Eigen::Vector2f dir, sp;
    calc_start_dir<float>(rhotheta, sp, dir);
    float t1 = -1e10, t2 = 1e10;
    for (auto it : points) {
        Eigen::Vector2f dvec = it->point - sp;
        float t = dvec.dot(dir);
        if (t > t1)
            t1 = t;
        if (t < t2)
            t2 = t;
    }
    start_ = sp + t1 * dir;
    end_ = sp + t2 * dir;
}

bool LineSegment::satisfy_param(float max_gap, float min_length) const {
    float vcos = cos(rhotheta[1]), vsin = sin(rhotheta[1]);
    Eigen::Vector2f dir(-vsin, vcos), sp(rhotheta[0] * vcos, rhotheta[0] * vsin);
    int N = points.size();
    vector<float> t_list(N);
    for (int i = 0; i < N; i++) {
        Eigen::Vector2f dvec = points[i]->point - sp;
        t_list[i] = dvec.dot(dir);
    }
    sort(t_list.begin(), t_list.end());
    if (t_list.back() - t_list.front() < min_length)
        return false;
    for (int i = 0; i < N - 1; i++) {
        if (t_list[i + 1] - t_list[i] >= max_gap)
            return false;
    }
    return true;
}

void LineSegmentExtractor::extract_lines(PointVector& points) {
    segments.clear();
    LineSegment segment(points);
    segment.leastSqFit(3.0, false);
    PBF(segment, 3.0);

    segments.erase(std::remove_if(segments.begin(), segments.end(), [this](const auto& seg) {
                       return !seg.satisfy_param(params.max_line_gap, 0.0);
                   }),
                   segments.end());

    merge2();
    // merge();

    for (auto& seg : segments)
        seg.leastSqFit(3.0, false, true);

    segments.erase(std::remove_if(segments.begin(), segments.end(), [this](const auto& seg) {
                       return seg.points.size() < params.min_line_points || !seg.satisfy_param(params.max_line_gap, params.min_line_length);
                   }),
                   segments.end());
    for (auto& seg : segments) {
        seg.projectEndpoints();
    }
}

void LineSegmentExtractor::merge() {
    if (segments.size() <= 1)
        return;
    // see paper Straight Line Segments Extraction and EKF-SLAM in Indoor Environment
    // http://www.joace.org/uploadfile/2014/0113/20140113054354731.pdf
    while (true) {
        for (int i = 0; i < segments.size(); i++) {
            auto& prev_seg = segments[i];
            for (int j = i + 1; j < segments.size(); j++) {
                const auto& cur_seg = segments[j];
                Eigen::Vector2f dL = cur_seg.rhotheta - prev_seg.rhotheta;                   // eq. 12
                float chiSq = dL.transpose() * (cur_seg.cov + prev_seg.cov).inverse() * dL;  // eq.  11
                if (chiSq < 3.0 && pairwise_closest(prev_seg.points, cur_seg.points) < params.max_line_gap) {
                    Eigen::Matrix2f prev_cov_inv = prev_seg.cov.inverse();

                    // eq. 13: compute the new covariance of the merged line
                    prev_seg.cov.noalias() = (cur_seg.cov.inverse() + prev_cov_inv).inverse();

                    // eq. 14: update parameters of the merged line
                    prev_seg.rhotheta = prev_seg.cov * (prev_cov_inv * prev_seg.rhotheta + cur_seg.cov.inverse() * cur_seg.rhotheta);
                    prev_seg.points.insert(prev_seg.points.end(), cur_seg.points.begin(), cur_seg.points.end());
                    prev_seg.uj.insert(prev_seg.uj.end(), cur_seg.uj.begin(), cur_seg.uj.end());
                    prev_seg.leastSqFit(3.0, false, true);
                    segments.erase(segments.begin() + j);
                    goto continue_outer;
                }
            }
        }
        break;
    continue_outer:;
    }
}

void LineSegmentExtractor::merge2() {
    struct Pair {
        float dist;
        int idx;
    };
    if (segments.size() <= 1)
        return;
    vector<Pair> rank;
    while (true) {
        int N = segments.size();
        rank.resize(N);
        for (int i = 0; i < N; i++) {
            auto& prev_seg = segments[i];
            Eigen::Vector2f xybar;
            prev_seg.calc_xybar(xybar, 3.0);

            for (int j = i + 1; j < N; j++) {
                const auto& cur_seg = segments[j];
                Eigen::Vector2f cur_xybar;
                cur_seg.calc_xybar(cur_xybar, 3.0);
                rank[j].dist = (cur_xybar - xybar).squaredNorm();
                rank[j].idx = j;
            }
            sort(rank.begin() + i + 1, rank.end(), [](auto& p1, auto& p2) {
                return p1.dist > p2.dist;
            });
            if (i + 1 < N - 1) {
                float disp[2] = {1e10, 1e10};
                LineSegment fused[2] = {prev_seg, prev_seg};
                for (int j = 0; j < 2; j++) {
                    const auto& target_seg = segments[rank[i + 1 + j].idx];
                    const auto& points2 = target_seg.points;
                    if (pairwise_closest(fused[j].points, points2) <= params.max_line_gap) {
                        fused[j].points.insert(fused[j].points.end(), points2.begin(), points2.end());
                        fused[j].uj.insert(fused[j].uj.end(), target_seg.uj.begin(), target_seg.uj.end());
                        // fill(fused[j].uj.begin(), fused[j].uj.end(), 1.0);
                        fused[j].leastSqFit(3.0, false);
                        disp[j] = fused[j].calc_dispersion();
                    }
                }
                if (min(disp[0], disp[1]) <= 0.06) {
                    int j = disp[0] > disp[1];
                    prev_seg = std::move(fused[j]);
                    segments.erase(segments.begin() + rank[i + 1 + j].idx);
                    goto continue_outer;
                }
            }
        }
        break;
    continue_outer:;
    }
}

// prototype based fuzzying
void LineSegmentExtractor::PBF(LineSegment& segment, float m) {
    int N = segment.points.size();
    if (N <= params.min_line_points)
        return;
    if (segment.calc_dispersion() <= 0.06) {
        // cout << segment.uj << endl;
        // fill(segment.uj.begin(), segment.uj.end(), 1.0);
        segment.leastSqFit(m, false, true);
        segments.push_back(segment);
        return;
    }

    // initialize 2 prototypes using the first 2 and last 2 points
    LineSegment segments[2] = {segment.points, segment.points};
    segments[0].rhotheta = topolar<float>(segment.points[0]->point, segment.points[1]->point);
    segments[1].rhotheta = topolar<float>(segment.points[N - 1]->point, segment.points[N - 2]->point);

    for (int i = 0; i < 2; i++) {
        Eigen::Vector2f xybar;
        segments[i].calc_xybar(xybar, m);
        segments[i].calc_dij(xybar);
    }

    // calculate line parameters until convergence
    for (int _ = 0; _ < 100; _++) {
        bool cont = true;
        for (int i = 0; i < 2; i++) {
            for (int j = 0; j < N; j++) {
                float new_val = 0.0, num = segments[i].dj[j];
                for (int k = 0; k < 2; k++) {
                    new_val += pow(num / segments[k].dj[j], 1.0f / (m - 1.0f));
                }
                new_val = 1.0f / new_val;
                float& old_val = segments[i].uj[j];
                if (abs(old_val - new_val) > 5e-4f)
                    cont = false;
                old_val = new_val;
            }
        }
        if (cont)
            goto converged;
        for (int i = 0; i < 2; i++)
            segments[i].leastSqFit(m);
    }
    cout << "not converged!!! abort." << endl;
    return;
converged:;
    for (int i = 0; i < 2; i++)
        segments[i].points.clear();

    for (int j = 0; j < N; j++) {
        auto pt = segment.points[j];
        int i = segments[0].uj[j] < segments[1].uj[j];
        auto& seg = segments[i];
        seg.uj[seg.points.size()] = seg.uj[j];
        seg.points.push_back(pt);
    }
    if (segments[0].points.size() == segment.points.size() || segments[1].points.size() == segment.points.size()) {
        cout << "not making any progress, abort" << endl;
        return;
    }
    for (int i = 0; i < 2; i++) {
        auto& seg = segments[i];
        seg.uj.resize(seg.points.size());
        seg.dj.clear();
        seg.dj.shrink_to_fit();
        PBF(seg);
    }
}
}  // namespace ls_extractor