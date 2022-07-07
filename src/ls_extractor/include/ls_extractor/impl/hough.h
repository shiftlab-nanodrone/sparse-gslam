#include <iostream>
#include <memory>
#include <unordered_set>
#include <vector>

#include "ls_extractor/defs.h"
#include "ls_extractor/utils.h"

namespace ls_extractor {
using namespace std;

using Vector2fVector = std::vector<Eigen::Vector2f>;

constexpr double max_theta = M_PI, min_theta = 0;
constexpr double max_rho = 4.0, min_rho = -4.0;
constexpr double theta_step = M_PI / 45, rho_step = 0.2;
constexpr int lines_max = 100, threshold = 25, w_size = 4;
constexpr float max_line_gap = 0.8, min_line_length = 0.6;

class LineSegmentExtractor {
   public:
    struct LineSegment {
        Eigen::Vector2f rhotheta, start, end;
        Eigen::Matrix2f cov;
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    };
    using SegmentVector = std::vector<LineSegment, Eigen::aligned_allocator<LineSegment>>;
    struct PointWithT {
        const ls_extractor::Point* point;
        float t;
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    };
    using PointWithTVec = std::vector<PointWithT, Eigen::aligned_allocator<PointWithT>>;
    struct SegmentWithPoint{
        LineSegment seg;
        PointWithTVec point_t;
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    };
    using SegmentWithPointVec = std::vector<SegmentWithPoint, Eigen::aligned_allocator<SegmentWithPoint>>;

    int num_angle, num_rho;
    std::vector<PPtrVector*> _accum;
    std::vector<PPtrVector> _store;
    std::vector<int> _sort_buf;
    Vector2fVector _trig_table;
    SegmentVector segments;
    SegmentWithPointVec seg_points;

    LineSegmentExtractor() : 
                             num_angle(floor((max_theta - min_theta) / theta_step)),
                             num_rho(ceil((max_rho - min_rho) / rho_step)),
                             _accum((num_angle + w_size * 2) * num_rho),
                             _store(num_angle * num_rho),
                             _trig_table(num_angle) {
        float ang = min_theta;
        float irho = 1 / rho_step;
        for (int n = 0; n < num_angle; ang += theta_step, n++) {
            _trig_table[n] = {cos(ang) * irho, sin(ang) * irho};
        }
        auto* accum = _accum.data() + num_rho * w_size;
        int sz = num_angle * num_rho;
        for (int i = 0; i < sz; i++) {
            accum[i] = &_store[i];
        }
        auto* src_ptr = accum + (num_angle - 1) * num_rho;
        for (int i = -w_size; i < 0; i++) {
            std::reverse_copy(src_ptr + i * num_rho, src_ptr + (i + 1) * num_rho, accum + i * num_rho);
        }
        for (int i = 1; i <= w_size; i++) {
            std::reverse_copy(accum + i * num_rho, accum + (i + 1) * num_rho, src_ptr + i * num_rho);
        }
    }

    void maxWindow(PPtrVector** accum, const int cr, const int cn) {
        int lb_n = cn - w_size, rb_n = cn + w_size + 1;
        int lb = std::max(0, cr - w_size), rb = std::min(num_rho, cr + w_size + 1);
        int max_r, max_n, max_v = 0;
        for (int n = lb_n; n < rb_n; n++) {
            for (int r = lb; r < rb; r++) {
                if (accum[n * num_rho + r]->size() > max_v) {
                    max_v = accum[n * num_rho + r]->size();
                }
            }
        }
        for (int n = lb_n; n < rb_n; n++) {
            for (int r = lb; r < rb; r++) {
                if (accum[n * num_rho + r]->size() < max_v) {
                    accum[n * num_rho + r]->clear();
                }
            }
        }
    }

    void findLocalMaximums(PPtrVector** accum, int threshold) {
        for (int n = 0; n < num_angle; n++) {
            for (int r = 0; r < num_rho; r++) {
                int idx = n * num_rho + r;
                if (accum[idx]->size() > threshold) {
                    maxWindow(accum, r, n);
                    // if (accum[idx]->size())
                    _sort_buf.push_back(idx);
                }
            }
        }
    }

    void projectEndpoints() {
        for(auto & seg : seg_points){
            float vcos = cos(seg.seg.rhotheta[1]), vsin = sin(seg.seg.rhotheta[1]);
            Eigen::Vector2f dir(-vsin, vcos), sp(seg.seg.rhotheta[0] * vcos, seg.seg.rhotheta[0] * vsin);
            seg.seg.start = sp + seg.point_t.front().t * dir;
            seg.seg.end = sp + seg.point_t.back().t * dir;
        }
    }

    void split_seg(const PPtrVector& points, LineSegment& seg){
        float vcos = cos(seg.rhotheta[1]), vsin = sin(seg.rhotheta[1]);
        Eigen::Vector2f dir(-vsin, vcos), sp(seg.rhotheta[0] * vcos, seg.rhotheta[0] * vsin);
        PointWithTVec point_list(points.size());
        for (int i = 0; i < points.size(); i++) {
            Eigen::Vector2f dvec = points[i]->point - sp;
            point_list[i].t = dvec.dot(dir);
            point_list[i].point = points[i];
        }
        sort(point_list.begin(), point_list.end(), [](auto a, auto b){return a.t < b.t;});

        std::vector<PointWithTVec> point_result;
        PointWithTVec temp;
        temp.push_back(point_list[0]);
        for (int i = 1; i < point_list.size(); i++) {
            if (point_list[i].t - point_list[i - 1].t < max_line_gap){
                temp.push_back(point_list[i]);
            }else{
                point_result.push_back(temp);
                temp.clear();
                temp.push_back(point_list[i]);
            }
        }
        if(temp.size() > threshold){
            point_result.push_back(temp);
        }
        for(int i = 0; i < point_result.size(); i++){
            temp = point_result[i];
            if (temp.back().t - temp.front().t <= min_line_length) continue;
            seg.start = sp + temp.front().t * dir;
            seg.end = sp + temp.back().t * dir;
            segments.push_back(seg);
            seg_points.push_back({seg, point_result[i]});
        }
    }

    void merge_overlap(){
        // std::cout << "size" << segments.size() << std::endl;
        std::unordered_set<int> index;
        for(int i = 0; i < seg_points.size(); i ++){
            for(int j = 0; j < seg_points.size(); j ++){
                if(i == j) continue;
                // project s2 endpoints onto s1
                auto s1 = seg_points[i];
                auto s2 = seg_points[j];

                float vcos1 = cos(s1.seg.rhotheta[1]), vsin1 = sin(s1.seg.rhotheta[1]);
                float vcos2 = cos(s2.seg.rhotheta[1]), vsin2 = sin(s2.seg.rhotheta[1]);
                Eigen::Vector2f dir_perp1(vcos1, vsin1), dir_para1(-vsin1, vcos1), dir_perp2(vcos2, vsin2), dir_para2(-vsin2, vcos2), \
                sp1(s1.seg.rhotheta[0] * vcos1, s1.seg.rhotheta[0] * vsin1), sp2(s2.seg.rhotheta[0] * vcos2, s2.seg.rhotheta[0] * vsin2);
                auto dvec_start = s2.seg.start - sp1;
                auto dvec_end = s2.seg.end - sp1;

                auto dist_perp_start = dvec_start.dot(dir_perp1);
                auto dist_perp_end = dvec_end.dot(dir_perp1);
                auto dist_para_start = dvec_start.dot(dir_para1);
                auto dist_para_end = dvec_end.dot(dir_para1);

                bool cond1 = abs(dist_perp_start + dist_perp_end) < 0.4; // there could be both ways
                bool cond2 = dist_para_start < s1.point_t.back().t + 0.15 && dist_para_start > s1.point_t.front().t - 0.15; // assume ordered 
                bool cond3 = dist_para_end < s1.point_t.back().t + 0.15 && dist_para_start > s1.point_t.front().t - 0.15; // assume ordered 
                if(cond1 && cond2 && cond3){
                    // use length and variance
                    float lengthi = (seg_points[i].seg.end - seg_points[i].seg.start).norm();
                    float lengthj = (seg_points[j].seg.end - seg_points[j].seg.start).norm();
                    
                    if(abs(lengthj - lengthi) < 0.1){
                        float var1 = 0, var2 = 0;
                        for(auto p : seg_points[i].point_t){
                            var1 += pow((p.point->point - sp1).dot(dir_perp1),2);
                        }
                        for(auto p : seg_points[j].point_t){
                            var2 += pow((p.point->point - sp2).dot(dir_perp2),2);
                        }
                        var1 = var1 / seg_points[i].point_t.size();
                        var2 = var2 / seg_points[j].point_t.size();

                        if(var1 > var2){
                            index.insert(i);
                        }else{
                            index.insert(j);
                        }
                    }else{
                        if(lengthi < lengthj){
                            index.insert(i);
                        }else{
                            index.insert(j);
                        }
                    }
                }
            }
        }
        SegmentWithPointVec temp_seg_points;
        SegmentVector temp_seg;
        for(int i = 0; i < segments.size(); i++){
            if(index.find(i) == index.end()){
                temp_seg_points.push_back(seg_points[i]);
                temp_seg.push_back(segments[i]);
            }
        }
        segments = temp_seg;
        seg_points = temp_seg_points;
    }

    void debug_view_endpoints(const PPtrVector& points, LineSegment& seg) {
        float vcos = cos(seg.rhotheta[1]), vsin = sin(seg.rhotheta[1]);
        Eigen::Vector2f dir(-vsin, vcos), sp(seg.rhotheta[0] * vcos, seg.rhotheta[0] * vsin);
        std::vector<float> t_list(points.size());
        for (int i = 0; i < points.size(); i++) {
            Eigen::Vector2f dvec = points[i]->point - sp;
            t_list[i] = dvec.dot(dir);
        }
        sort(t_list.begin(), t_list.end());

        seg.start = sp + t_list.front() * dir;
        seg.end = sp + t_list.back() * dir;
        segments.push_back(seg);
    }

    void leastSqFit() {
        // eq. 11: compute least square line parameters
        segments.clear();
        for(auto& seg : seg_points){
            Eigen::Vector2f xybar = Eigen::Vector2f::Zero(), Sx2y2 = Eigen::Vector2f::Zero();
            float Sxy = 0.0;
            for(auto p : seg.point_t){
                xybar += p.point->point;                            // accumulate sum (xi), sum (yi)
                Sxy += p.point->point[0] * p.point->point[1];       // accumulate sum (xi yi)
                Sx2y2 += p.point->point.array().square().matrix();  // accumulate sum (xi^2), sum (yi^2)
            }
            int n = seg.point_t.size();
            xybar /= n;                                   // xbar = sum(xi)/n, ybar = sum(yi)/n
            Sx2y2 -= n * xybar.array().square().matrix(); // xbar = sum(xi)/n, ybar = sum(yi)/n
            Sxy -= n * xybar[0] * xybar[1];               // sum (xi-xbar)(yi-ybar) = sum(xi yi) - n*xbar*ybar
            float Sy2_Sx2 = Sx2y2[1] - Sx2y2[0];

            seg.seg.rhotheta[1] = 0.5 * atan2(-2 * Sxy, Sy2_Sx2);
            // make sure that rho >= 0
            float ct = cos(seg.seg.rhotheta[1]), st = sin(seg.seg.rhotheta[1]);
            seg.seg.rhotheta[0] = xybar[0] * ct + xybar[1] * st;
            if (seg.seg.rhotheta[0] < 0.0) {
                seg.seg.rhotheta[0] = -seg.seg.rhotheta[0];
                seg.seg.rhotheta[1] += M_PI;
                if (seg.seg.rhotheta[1] > M_PI) seg.seg.rhotheta[1] -= 2 * M_PI;
            }
            
            ct = cos(seg.seg.rhotheta[1]), st = sin(seg.seg.rhotheta[1]);
            float xbar_st = xybar[0] * st, ybar_ct = xybar[1] * ct;
            seg.seg.cov = Eigen::Matrix2f::Zero();
            float denum = 1.0 / (Sy2_Sx2 * Sy2_Sx2 + 4 * Sxy * Sxy);
            float ct_n = ct / n, st_n = st / n;
            for (auto p : seg.point_t) {
                Eigen::Matrix2f Ai;
                Eigen::Vector2f d = xybar - p.point->point;

                // compute the Jacobian for each point
                Ai(1, 0) = (d[1] * Sy2_Sx2 + 2 * Sxy * d[0]) * denum;
                Ai(1, 1) = (d[0] * Sy2_Sx2 - 2 * Sxy * d[1]) * denum;
                Ai(0, 0) = ct_n - xbar_st * Ai(1, 0) + ybar_ct * Ai(1, 0);
                Ai(0, 1) = st_n - xbar_st * Ai(1, 1) + ybar_ct * Ai(1, 1);
                seg.seg.cov += Ai * p.point->cov * Ai.transpose();
            }

            // project end point 
            float vcos = cos(seg.seg.rhotheta[1]), vsin = sin(seg.seg.rhotheta[1]);
            Eigen::Vector2f dir(-vsin, vcos), sp(seg.seg.rhotheta[0] * vcos, seg.seg.rhotheta[0] * vsin);

            PointWithTVec point_list(seg.point_t.size());
            for (int i = 0; i < seg.point_t.size(); i++) {
                Eigen::Vector2f dvec = seg.point_t[i].point->point - sp;
                point_list[i].t = dvec.dot(dir);
                point_list[i].point = seg.point_t[i].point;
            }
            sort(point_list.begin(), point_list.end(), [](auto a, auto b){return a.t < b.t;});

            seg.seg.start = sp + point_list.front().t * dir;
            seg.seg.end = sp + point_list.back().t * dir;
            seg.point_t = point_list;
            
            segments.push_back(seg.seg);
        }
    }

    void extract_lines(const PointVector& points) {
        segments.clear();
        seg_points.clear();
        _sort_buf.clear();
        for (auto& s : _store)
            s.clear();
        // stage 1. fill accumulator
        float irho_min = min_rho / rho_step;
        auto* accum = _accum.data() + num_rho * w_size;
        for (const auto& pt : points)
            for (int n = 0; n < num_angle; n++) {
                int r = round(pt.point.dot(_trig_table[n]) -
                              irho_min);
                int idx = n * num_rho + r;
                if (idx >= (num_rho * num_angle) || idx < 0) {
                    // std::cout << "out of bound " << pt.point << ' ' << idx << std::endl;
                    continue;
                }
                accum[idx]->push_back(&pt);
            }

        findLocalMaximums(accum, threshold);

        int lines_max = _sort_buf.size();
        for (int i = 0; i < lines_max; i++) {
            int idx = _sort_buf[i];
            if (!accum[idx]->size())
                continue;
            LineSegment seg;
            seg.rhotheta = {
                min_rho + (idx % num_rho) * rho_step,
                min_theta + (idx / num_rho) * theta_step};

            // debug_view_endpoints(*accum[idx], seg);
            split_seg(*accum[idx], seg);
        }
       
        merge_overlap();
        leastSqFit();
        // segments.erase(segments.end()-3);
        // segments.erase(segments.end()-8);
        // std::cout << "spoint size" <<seg_points[segments.size()-3].point_t.size() << std::endl;
    }
};
}  // namespace ls_extractor