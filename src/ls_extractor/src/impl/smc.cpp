#include "ls_extractor/impl/smc.h"

#include <Eigen/Dense>
#include <boost/pending/disjoint_sets.hpp>

#include "ls_extractor/utils.h"

namespace ls_extractor {
using namespace std;

void LineSegment::endpointFit() {
    rhotheta = topolar<float>(start->point, (end - 1)->point);
}
float LineSegment::gapBetween(const LineSegment& other) const {
    pit arr1[] = {start, end - 1};
    pit arr2[] = {other.start, other.end - 1};
    float dist = 1e10;
    for (int i = 0; i < 2; i++) {
        for (int j = 0; j < 2; j++) {
            float d = (arr1[i]->point - arr2[j]->point).squaredNorm();
            if (d > dist)
                dist = d;
        }
    }
    return sqrt(dist);
}

// see Mobile robot SLAM for line-based environment representation (appendix)
// https://folk.ntnu.no/skoge/prost/proceedings/cdc-ecc05/pdffiles/papers/2714.pdf
void LineSegment::leastSqFit() {
    // eq. 11: compute least square line parameters
    Eigen::Vector2f xybar = Eigen::Vector2f::Zero(), Sx2y2 = Eigen::Vector2f::Zero();
    float Sxy = 0.0;
    for (pit it = start; it != end; it++) {
        xybar += it->point;              // accumulate sum (xi), sum (yi)
        Sxy += it->point.prod();         // accumulate sum (xi yi)
        Sx2y2 += it->point.cwiseAbs2();  // accumulate sum (xi^2), sum (yi^2)
    }
    int n = end - start;
    xybar /= n;                      // xbar = sum(xi)/n, ybar = sum(yi)/n
    Sx2y2 -= n * xybar.cwiseAbs2();  // sum (xi-xbar)^2 = sum (xi^2) - n*xbar^2, sum (yi-ybar)^2 = sum (yi^2) - n*ybar^2
    Sxy -= n * xybar.prod();         // sum (xi-xbar)(yi-ybar) = sum(xi yi) - n*xbar*ybar

    float Sy2_Sx2 = Sx2y2[1] - Sx2y2[0];
    rhotheta[1] = 0.5 * atan2(-2 * Sxy, Sy2_Sx2);

    float ct = cos(rhotheta[1]), st = sin(rhotheta[1]);
    rhotheta[0] = xybar[0] * ct + xybar[1] * st;
    checkRhoTheta<float>(rhotheta);  // make sure that rho >= 0

    ct = cos(rhotheta[1]), st = sin(rhotheta[1]);
    // compute covariance matrix of the line parameters
    float xbar_st = xybar[0] * st, ybar_ct = xybar[1] * ct;
    cov.setZero();
    float denum = 1.0 / (Sy2_Sx2 * Sy2_Sx2 + 4 * Sxy * Sxy);
    float ct_n = ct / n, st_n = st / n;
    for (pit it = start; it != end; it++) {
        Eigen::Matrix2f Ai;
        Eigen::Vector2f d = xybar - it->point;

        // compute the Jacobian for each point
        Ai(1, 0) = (d[1] * Sy2_Sx2 + 2 * Sxy * d[0]) * denum;
        Ai(1, 1) = (d[0] * Sy2_Sx2 - 2 * Sxy * d[1]) * denum;
        Ai(0, 0) = ct_n - xbar_st * Ai(1, 0) + ybar_ct * Ai(1, 0);
        Ai(0, 1) = st_n - xbar_st * Ai(1, 1) + ybar_ct * Ai(1, 1);
        cov.noalias() += Ai * it->cov * Ai.transpose();
    }
}

void LineSegment::projectEndpoints() {
    Eigen::Vector2f sp, dir, tout;
    calc_start_dir<float>(rhotheta, sp, dir);
    calc_endpoints<float>(sp, dir, start->point, (end - 1)->point, tout);
    start_ = sp + tout[0] * dir;
    end_ = sp + tout[1] * dir;
}

unordered_map<int, PointVector> LineSegmentExtractor::cluster(const PointVector& points) {
    ranks.resize(points.size());
    parents.resize(points.size());
    boost::disjoint_sets<int*, int*> set(ranks.data(), parents.data());
    for (int i = 0; i < points.size(); i++)
        set.make_set(i);
    const float cutoff_distance = params.cluster_threshold * params.cluster_threshold;
    for (int i = 0; i < points.size(); i++) {
        for (int j = i + 1; j < points.size(); j++) {
            if ((points[i].point - points[j].point).squaredNorm() <= cutoff_distance)
                set.union_set(i, j);
        }
    }

    // return map
    unordered_map<int, PointVector> map;
    for (int i = 0; i < points.size(); i++) {
        map[set.find_set(i)].push_back(points[i]);
    }
    return map;
}

void LineSegmentExtractor::extract_lines(PointVector& points) {
    segments.clear();
    auto map = cluster(points);
    for (auto& p : map) {
        if (p.second.size() >= params.min_line_points) {
            // Eigen::Vector2f cent = Eigen::Vector2f::Zero();
            // for (const auto& pt : p.second) {
            //     cent += pt.point;
            // }
            // cent /= p.second.size();
            // for (auto& pt : p.second) {
            //     Eigen::Vector2f sp = pt.point - cent;
            //     pt.rhotheta[1] = atan2(sp[1], sp[0]);
            // }
            // sort(p.second.begin(), p.second.end(), [](const auto& p1, const auto& p2) { return p1.rhotheta[1] < p2.rhotheta[1]; });
            // for (auto& pt : p.second) {
            //     pt.rhotheta[1] = atan2(pt.point[1], pt.point[0]);
            // }
            extract_lines_helper(p.second);
        }
    }
    if (map.size() == 1)
        merge();
    else
        merge_pairwise();
    for (auto& seg : segments)
        seg.projectEndpoints();
}

void LineSegmentExtractor::extract_lines_helper(PointVector& points) {
    sort(points.begin(), points.end(), [](const auto& p1, const auto& p2) { return p1.rhotheta[1] < p2.rhotheta[1]; });

    int fidx = 1;
    for (int i = 1; i < (int)(points.size() - 1); i++) {
        const auto& pt1 = points[i - 1];
        const auto& pt2 = points[i];
        const auto& pt3 = points[i + 1];

        if (abs(pt1.rhotheta[0] - pt2.rhotheta[0]) >= params.outlier_dist && abs(pt3.rhotheta[0] - pt2.rhotheta[0]) >= params.outlier_dist) {
            LineSegment line(points.begin() + i - 1, points.begin() + i + 1);
            if (line.distToPoint(pt2) > params.min_split_dist)
                continue;
        }
        points[fidx++] = pt2;
    }
    points[fidx++] = points.back();
    points.resize(fidx);

    // splits.resize(0);
    int cur_size = segments.size();
    split(points.cbegin(), points.cend());
    segments.erase(std::remove_if(segments.begin() + cur_size, segments.end(), [this](const auto& seg) {
                       return seg.end - seg.start < params.min_line_points || seg.length() < params.min_line_length;
                   }),
                   segments.end());

    for (auto& seg : segments)
        seg.leastSqFit();
}

void LineSegmentExtractor::split(pit start, pit end) {
    // Don't split if only a single point (only occurs when orphaned by gap)
    if (end - start <= 1)
        return;

    LineSegment line(start, end);
    line.endpointFit();

    float dist_max = 0;
    float gap_max = (start->point - (start + 1)->point).norm();
    pit i_max, i_gap = start;

    // Find the farthest point and largest gap
    for (pit it = start + 1; it != (end - 1); it++) {
        float dist = line.distToPoint(*it);
        if (dist > dist_max) {
            dist_max = dist;
            i_max = it;
        }
        float gap = (it->point - (it + 1)->point).norm();
        if (gap > gap_max) {
            gap_max = gap;
            i_gap = it;
        }
    }

    // Check if line meets requirements or should be split
    if (dist_max < params.min_split_dist && gap_max < params.max_line_gap) {
        segments.push_back(line);
    } else {
        pit i_split = dist_max >= params.min_split_dist ? i_max : i_gap + 1;

        // splits.push_back(i_split);
        split(start, i_split);
        split(i_split, end);
    }
}

void LineSegmentExtractor::merge() {
    if (segments.size() <= 1)
        return;
    // see paper Straight Line Segments Extraction and EKF-SLAM in Indoor Environment
    // http://www.joace.org/uploadfile/2014/0113/20140113054354731.pdf
    int fidx = 0;
    for (int i = 1; i < segments.size(); i++) {
        auto& prev_seg = segments[fidx];
        const auto& cur_seg = segments[i];
        Eigen::Vector2f dL = cur_seg.rhotheta - prev_seg.rhotheta;                   // eq. 12
        float chiSq = dL.transpose() * (cur_seg.cov + prev_seg.cov).inverse() * dL;  // eq.  11
        if (chiSq < 4.605f && prev_seg.gapBetween(cur_seg) <= params.max_line_gap) {
            Eigen::Matrix2f prev_cov_inv = prev_seg.cov.inverse();

            // eq. 13: compute the new covariance of the merged line
            prev_seg.cov.noalias() = (cur_seg.cov.inverse() + prev_cov_inv).inverse();

            // eq. 14: update parameters of the merged line
            prev_seg.rhotheta = prev_seg.cov * (prev_cov_inv * prev_seg.rhotheta + cur_seg.cov.inverse() * cur_seg.rhotheta);
            prev_seg.end = cur_seg.end;
        } else {
            segments[++fidx] = cur_seg;
        }
    }
    segments.resize(fidx + 1);
}

void LineSegmentExtractor::merge_pairwise() {
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
                if (chiSq < 4.605f && prev_seg.gapBetween(cur_seg) < params.max_line_gap) {
                    Eigen::Matrix2f prev_cov_inv = prev_seg.cov.inverse();

                    // eq. 13: compute the new covariance of the merged line
                    prev_seg.cov.noalias() = (cur_seg.cov.inverse() + prev_cov_inv).inverse();

                    // eq. 14: update parameters of the merged line
                    prev_seg.rhotheta = prev_seg.cov * (prev_cov_inv * prev_seg.rhotheta + cur_seg.cov.inverse() * cur_seg.rhotheta);
                    prev_seg.leastSqFit();
                    segments.erase(segments.begin() + j);
                    goto continue_outer;
                }
            }
        }
        break;
    continue_outer:;
    }
}

}  // namespace ls_extractor