#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/video/tracking.hpp>
#include <cmath>
#include <vector>
#include <algorithm>
#include <numeric>
#include <cstdint>

#if defined(__ANDROID__) || defined(__linux__)
    #define FFI_EXPORT extern "C" __attribute__((visibility("default"))) __attribute__((used))
#else
    #define FFI_EXPORT extern "C"
#endif

// ============================================================================
// 1. FFI 공유 데이터 구조체 (Dart <-> C++ Zero-Copy 텔레메트리 규격)
// ============================================================================
#pragma pack(push, 1)
struct TelemetryOutput {
    int32_t status_code;          // 0: 대기, 1: 록온 추적중, 2: 측정 완료, -1: 추적 유실 경고
    double instant_draft_m;       // 현재 프레임 순간 흘수 Z*(t) [m]
    float waterline_u;            // 검출된 수면-스파인 교차점 X 픽셀
    float waterline_v;            // 검출된 수면-스파인 교차점 Y 픽셀
    int32_t active_mark_count;    // 현재 추적 중인 마크 개수 (최대 16개)
    float mark_u[16];             // 각 마크의 현재 X 픽셀 좌표
    float mark_v[16];             // 각 마크의 현재 Y 픽셀 좌표
    float mark_z[16];             // 각 마크의 물리 흘수 값 (예: 8.0, 8.2, 8.4...)
    int32_t spine_pts_count;      // AR 곡선 렌더링용 스파인 샘플 수 (50개)
    float spine_u[50];            // 스파인 궤적 X 픽셀 배열
    float spine_v[50];            // 스파인 궤적 Y 픽셀 배열
    float cam_roll_deg;           // IMU 카메라 Roll 각도 (deg)
    float cam_pitch_deg;          // IMU 카메라 Pitch 각도 (deg)
    int32_t collected_frames;     // 현재까지 누적된 유효 파형 샘플 수
};

struct FinalSurveyResult {
    int32_t is_valid;             // 1: 정상 산출, 0: 샘플 부족/실패
    double true_draft_m;          // 최종 보정 참 흘수 (AWLS + Sainflou 차감 완료) [m]
    double apparent_mean_m;       // 차감 전 외견 중심 흘수 [m]
    double simple_mean_m;         // 단순 산술 평균 흘수 (비교용) [m]
    double sainflou_deduct_mm;    // Sainflou 중복파 수위 상승 차감량 [mm]
    double swell_height_m;        // 추정 평균 너울 파고 H [m]
    double swell_period_s;        // 추정 우세 너울 주기 T_p [s]
    double observed_cycles;       // 관측된 총 너울 주기 횟수
    double trough_rmse_mm;        // 파저 구간 피팅 잔차 RMSE [mm]
    double split_half_diff_mm;    // 전반부/후반부 독립 연산 편차 [mm] (10mm 이내 합격)
    int32_t reliability_Pass;     // 무결성 최종 합격 여부 (1: PASS, 0: WARNING)
};
#pragma pack(pop)

// ============================================================================
// 2. 도면 없는 곡면 보간을 위한 C++ 단조 3차 에르미트 스플라인 (Monotone PCHIP)
// ============================================================================
class MonotonePCHIP {
public:
    std::vector<double> x_, y_, m_;

    bool build(const std::vector<double>& x, const std::vector<double>& y) {
        size_t n = x.size();
        if (n < 2) return false;
        x_ = x;
        y_ = y;
        m_.assign(n, 0.0);

        if (n == 2) {
            double d = (y[1] - y[0]) / (x[1] - x[0] + 1e-9);
            m_[0] = m_[1] = d;
            return true;
        }

        std::vector<double> h(n - 1), delta(n - 1);
        for (size_t i = 0; i < n - 1; ++i) {
            h[i] = std::max(1e-6, x[i + 1] - x[i]);
            delta[i] = (y[i + 1] - y[i]) / h[i];
        }

        // 내부 노드 Fritsch-Carlson 가중 조화 평균 기울기 계산 (오버슈트 원천 차단)
        for (size_t i = 1; i < n - 1; ++i) {
            if (delta[i - 1] * delta[i] <= 0.0) {
                m_[i] = 0.0;
            } else {
                double w1 = 2.0 * h[i] + h[i - 1];
                double w2 = h[i] + 2.0 * h[i - 1];
                m_[i] = (w1 + w2) / (w1 / delta[i - 1] + w2 / delta[i]);
            }
        }

        // 끝점 기울기
        m_[0] = ((2.0 * h[0] + h[1]) * delta[0] - h[0] * delta[1]) / (h[0] + h[1]);
        if (m_[0] * delta[0] <= 0.0) m_[0] = 0.0;
        else if ((delta[0] * delta[1] <= 0.0) && (std::abs(m_[0]) > 3.0 * std::abs(delta[0])))
            m_[0] = 3.0 * delta[0];

        m_[n - 1] = ((2.0 * h[n - 2] + h[n - 3]) * delta[n - 2] - h[n - 2] * delta[n - 3]) / (h[n - 2] + h[n - 3]);
        if (m_[n - 1] * delta[n - 2] <= 0.0) m_[n - 1] = 0.0;
        else if ((delta[n - 2] * delta[n - 3] <= 0.0) && (std::abs(m_[n - 1]) > 3.0 * std::abs(delta[n - 2])))
            m_[n - 1] = 3.0 * delta[n - 2];

        return true;
    }

    double eval(double xq) const {
        if (x_.empty()) return 0.0;
        size_t n = x_.size();
        if (xq <= x_.front()) {
            return y_.front() + m_.front() * (xq - x_.front());
        }
        if (xq >= x_.back()) {
            return y_.back() + m_.back() * (xq - x_.back());
        }

        auto it = std::upper_bound(x_.begin(), x_.end(), xq);
        size_t idx = std::max<size_t>(0, std::min<size_t>(n - 2, std::distance(x_.begin(), it) - 1));

        double h = x_[idx + 1] - x_[idx];
        double t = (xq - x_[idx]) / h;
        double t2 = t * t;
        double t3 = t2 * t;

        double h00 = 2.0 * t3 - 3.0 * t2 + 1.0;
        double h10 = t3 - 2.0 * t2 + t;
        double h01 = -2.0 * t3 + 3.0 * t2;
        double h11 = t3 - t2;

        return h00 * y_[idx] + h10 * h * m_[idx] + h01 * y_[idx + 1] + h11 * h * m_[idx + 1];
    }
};

// ============================================================================
// 3. 마크별 2D 등속도 칼만 필터 + LK 트랙 구조체
// ============================================================================
struct MarkTrack {
    double z_meter;
    cv::Point2f uv;
    cv::KalmanFilter kf;
    int occlusion_count;

    MarkTrack(double z, cv::Point2f init_pt) : z_meter(z), uv(init_pt), occlusion_count(0) {
        kf.init(4, 2, 0, CV_32F);
        kf.transitionMatrix = (cv::Mat_<float>(4, 4) <<
            1, 0, 1, 0,
            0, 1, 0, 1,
            0, 0, 1, 0,
            0, 0, 0, 1);
        cv::setIdentity(kf.measurementMatrix);
        cv::setIdentity(kf.processNoiseCov, cv::Scalar::all(0.05f));
        cv::setIdentity(kf.measurementNoiseCov, cv::Scalar::all(1.2f));
        cv::setIdentity(kf.errorCovPost, cv::Scalar::all(1.0f));
        kf.statePost.at<float>(0) = init_pt.x;
        kf.statePost.at<float>(1) = init_pt.y;
        kf.statePost.at<float>(2) = 0.0f;
        kf.statePost.at<float>(3) = 0.0f;
    }

    cv::Point2f predict() {
        cv::Mat p = kf.predict();
        return cv::Point2f(p.at<float>(0), p.at<float>(1));
    }

    cv::Point2f correct(cv::Point2f meas) {
        cv::Mat m = (cv::Mat_<float>(2, 1) << meas.x, meas.y);
        cv::Mat c = kf.correct(m);
        uv = cv::Point2f(c.at<float>(0), c.at<float>(1));
        return uv;
    }
};

// ============================================================================
// 4. 메인 엔진 클래스 (Bootstrap + Spine Tracking + AWLS Swell Solver)
// ============================================================================
class NativeDraftEngine {
public:
    double fps_ = 30.0;
    double flare_rad_ = 30.0 * M_PI / 180.0;
    double v_drain_ = 0.18;
    bool is_locked_ = false;

    cv::Mat prev_gray_;
    std::vector<MarkTrack> tracks_;
    std::vector<double> z_time_series_;

    NativeDraftEngine(double fps, double flare_deg) : fps_(fps) {
        flare_rad_ = flare_deg * M_PI / 180.0;
    }

    // ------------------------------------------------------------------------
    // [모듈 A] 터치 좌표 주변 Sobel 수평 에지 서브픽셀 스내핑 (마크 밑단 정밀 밀착)
    // ------------------------------------------------------------------------
    cv::Point2f snap_to_mark_bottom(const cv::Mat& gray, cv::Point2f seed_pt) {
        int r_x = 16, r_y = 22;
        int x0 = std::max(2, (int)seed_pt.x - r_x);
        int x1 = std::min(gray.cols - 3, (int)seed_pt.x + r_x);
        int y0 = std::max(2, (int)seed_pt.y - r_y);
        int y1 = std::min(gray.rows - 3, (int)seed_pt.y + r_y);

        if (x1 <= x0 + 4 || y1 <= y0 + 4) return seed_pt;

        cv::Rect roi(x0, y0, x1 - x0, y1 - y0);
        cv::Mat patch = gray(roi);
        cv::Mat grad_y;
        cv::Sobel(patch, grad_y, CV_32F, 0, 1, 3);

        // 각 행(Row)의 평균 에지 강도 계산 (가우시안 중심 가중치 부여)
        int best_r = patch.rows / 2;
        float max_score = -1.0f;
        std::vector<float> row_scores(patch.rows, 0.0f);

        for (int r = 1; r < patch.rows - 1; ++r) {
            float sum_g = 0.0f;
            for (int c = 0; c < patch.cols; ++c) {
                sum_g += std::abs(grad_y.at<float>(r, c));
            }
            float dist_penalty = std::exp(-0.004f * std::pow(r - patch.rows / 2.0f, 2));
            row_scores[r] = (sum_g / patch.cols) * dist_penalty;
            if (row_scores[r] > max_score) {
                max_score = row_scores[r];
                best_r = r;
            }
        }

        // 3점 포물선 서브픽셀 보간
        float sub_r = (float)best_r;
        if (best_r >= 1 && best_r < patch.rows - 1) {
            float g0 = row_scores[best_r - 1];
            float g1 = row_scores[best_r];
            float g2 = row_scores[best_r + 1];
            float denom = g0 - 2.0f * g1 + g2;
            if (std::abs(denom) > 1e-5f) {
                sub_r += 0.5f * (g0 - g2) / denom;
            }
        }
        return cv::Point2f(seed_pt.x, (float)y0 + sub_r);
    }

    // ------------------------------------------------------------------------
    // [모듈 B] 모델리스 부트스트랩 (터치 시드 -> PCHIP 스파인 생성 -> 0.2m 격자 자동 증식)
    // ------------------------------------------------------------------------
    bool bootstrap_from_seeds(const cv::Mat& gray, int count, const float* u_arr, const float* v_arr, const float* z_arr) {
        if (count < 2) return false;
        tracks_.clear();
        z_time_series_.clear();

        struct SeedItem { double z; cv::Point2f pt; };
        std::vector<SeedItem> seeds;
        for (int i = 0; i < count; ++i) {
            cv::Point2f snapped = snap_to_mark_bottom(gray, cv::Point2f(u_arr[i], v_arr[i]));
            seeds.push_back({(double)z_arr[i], snapped});
        }

        // 물리 흘수 Z 기준 오름차순 정렬 (낮은 흘수 -> 높은 흘수)
        std::sort(seeds.begin(), seeds.end(), [](const SeedItem& a, const SeedItem& b) {
            return a.z < b.z;
        });

        std::vector<double> z_seeds, u_seeds, v_seeds;
        for (const auto& s : seeds) {
            z_seeds.push_back(s.z);
            u_seeds.push_back(s.pt.x);
            v_seeds.push_back(s.pt.y);
        }

        // 시드 점들로부터 초기 PCHIP 곡선 생성
        MonotonePCHIP pchip_u, pchip_v;
        pchip_u.build(z_seeds, u_seeds);
        pchip_v.build(z_seeds, v_seeds);

        // 최저~최고 시드 범위(+하방 0.4m 여유) 내에서 0.2m 표준 메트릭 간격으로 마크 트랙 자동 생성!
        double z_min = std::floor(z_seeds.front() * 5.0 + 1e-4) / 5.0 - 0.2;
        double z_max = std::ceil(z_seeds.back() * 5.0 - 1e-4) / 5.0 + 0.2;

        for (double z = z_min; z <= z_max + 0.05; z += 0.2) {
            double z_round = std::round(z * 10.0) / 10.0;
            float u_interp = (float)pchip_u.eval(z_round);
            float v_interp = (float)pchip_v.eval(z_round);

            if (u_interp >= 15 && u_interp < gray.cols - 15 && v_interp >= 15 && v_interp < gray.rows - 15) {
                cv::Point2f refined = snap_to_mark_bottom(gray, cv::Point2f(u_interp, v_interp));
                tracks_.emplace_back(z_round, refined);
                if (tracks_.size() >= 16) break;
            }
        }

        prev_gray_ = gray.clone();
        is_locked_ = (tracks_.size() >= 2);
        return is_locked_;
    }

    // ------------------------------------------------------------------------
    // [모듈 C] 매 프레임 실시간 처리 (IMU 중력 정렬 + LK-칼만 추적 + 1D 서브픽셀 수면 추출)
    // ------------------------------------------------------------------------
    void process_frame(const cv::Mat& curr_gray, float gx, float gy, float gz, TelemetryOutput* out) {
        // 1. IMU 중력 벡터 기반 카메라 Roll / Pitch 각도 계산
        float g_norm = std::sqrt(gx * gx + gy * gy + gz * gz) + 1e-6f;
        float nx = gx / g_norm, ny = gy / g_norm, nz = gz / g_norm;
        float roll_rad = std::atan2(nx, ny);  // 세로/가로 파지 시 화면 내 중력 회전각
        float pitch_deg = std::asin(std::clamp(nz, -1.0f, 1.0f)) * 180.0f / (float)M_PI;

        out->cam_roll_deg = roll_rad * 180.0f / (float)M_PI;
        out->cam_pitch_deg = pitch_deg;

        if (!is_locked_ || prev_gray_.empty() || tracks_.size() < 2) {
            out->status_code = 0;
            out->active_mark_count = 0;
            return;
        }

        // 2. Forward-Backward Lucas-Kanade 광학 흐름 수행
        std::vector<cv::Point2f> p0, p1, p0_rev;
        for (auto& trk : tracks_) p0.push_back(trk.uv);

        std::vector<uchar> status, status_rev;
        std::vector<float> err, err_rev;
        cv::Size win_size(21, 21);
        cv::TermCriteria term(cv::TermCriteria::COUNT | cv::TermCriteria::EPS, 25, 0.01);

        cv::calcOpticalFlowPyrLK(prev_gray_, curr_gray, p0, p1, status, err, win_size, 3, term);
        cv::calcOpticalFlowPyrLK(curr_gray, prev_gray_, p1, p0_rev, status_rev, err_rev, win_size, 3, term);

        // 3. 광학 흐름 성공(Inlier) 마크 선별 및 전역 중력 정렬 이동량(Rigid Consensus) 추정
        std::vector<double> valid_z, valid_u, valid_v;
        std::vector<cv::Point2f> shift_vecs;

        for (size_t i = 0; i < tracks_.size(); ++i) {
            tracks_[i].predict();
            float fb_dist = cv::norm(p0[i] - p0_rev[i]);
            bool ok = (status[i] == 1) && (status_rev[i] == 1) && (fb_dist < 1.8f);
            if (ok) {
                shift_vecs.push_back(p1[i] - p0[i]);
                valid_z.push_back(tracks_[i].z_meter);
                valid_u.push_back(p1[i].x);
                valid_v.push_back(p1[i].y);
            }
        }

        // 중앙값 이동 벡터 계산 (물보라로 가려진 마크의 가상 코스팅 유도용)
        cv::Point2f med_shift(0.0f, 0.0f);
        if (!shift_vecs.empty()) {
            std::vector<float> dx_list, dy_list;
            for (const auto& s : shift_vecs) { dx_list.push_back(s.x); dy_list.push_back(s.y); }
            std::sort(dx_list.begin(), dx_list.end());
            std::sort(dy_list.begin(), dy_list.end());
            med_shift = cv::Point2f(dx_list[dx_list.size() / 2], dy_list[dy_list.size() / 2]);
        }

        // 4. 유효 마크들로 현 프레임의 PCHIP 가상 스파인(Virtual Guidance) 재구축
        MonotonePCHIP guide_u, guide_v;
        bool has_guide = (valid_z.size() >= 2) && guide_u.build(valid_z, valid_u) && guide_v.build(valid_z, valid_v);

        // 각 마크 칼만 필터 갱신 (가려진 마크는 PCHIP 가상 스파인 위치로 코스팅)
        std::vector<double> final_z, final_u, final_v;
        for (size_t i = 0; i < tracks_.size(); ++i) {
            float fb_dist = cv::norm(p0[i] - p0_rev[i]);
            bool ok = (status[i] == 1) && (fb_dist < 1.8f);
            cv::Point2f target_meas;

            if (ok && cv::norm((p1[i] - p0[i]) - med_shift) < 12.0f) {
                target_meas = p1[i];
                tracks_[i].occlusion_count = 0;
            } else if (has_guide) {
                target_meas = cv::Point2f((float)guide_u.eval(tracks_[i].z_meter),
                                          (float)guide_v.eval(tracks_[i].z_meter));
                tracks_[i].occlusion_count++;
            } else {
                target_meas = p0[i] + med_shift;
                tracks_[i].occlusion_count++;
            }

            cv::Point2f corr = tracks_[i].correct(target_meas);
            final_z.push_back(tracks_[i].z_meter);
            final_u.push_back(corr.x);
            final_v.push_back(corr.y);
        }

        // 5. IMU 중력 정렬 호 길이(Arc-Length) PCHIP 스파인 구축 및 하방 외삽(수면 탐색 범위 확장)
        MonotonePCHIP spine_u_of_z, spine_v_of_z;
        spine_u_of_z.build(final_z, final_u);
        spine_v_of_z.build(final_v.size() >= 2 ? final_z : valid_z, final_v);

        double z_low = final_z.front() - 0.8;  // 최하단 마크 아래 80cm까지 스파인 연장
        double z_high = final_z.back() + 0.2;

        // 6. 스파인 궤적을 따라 1mm(또는 5mm) 간격으로 픽셀 밝기 프로파일 샘플링 -> 수면 경계 검출
        int n_samples = 240;
        std::vector<double> z_grid(n_samples), u_grid(n_samples), v_grid(n_samples), intensity(n_samples, 0.0);

        for (int k = 0; k < n_samples; ++k) {
            double zk = z_low + (z_high - z_low) * k / (double)(n_samples - 1);
            double uk = spine_u_of_z.eval(zk);
            double vk = spine_v_of_z.eval(zk);
            z_grid[k] = zk;
            u_grid[k] = uk;
            v_grid[k] = vk;

            // 스파인 법선 방향으로 폭 ±5px 평균 휘도 추출 (노이즈 억제)
            int ix = std::clamp((int)std::round(uk), 5, curr_gray.cols - 6);
            int iy = std::clamp((int)std::round(vk), 2, curr_gray.rows - 3);
            float accum = 0.0f;
            for (int dx = -4; dx <= 4; ++dx) {
                accum += curr_gray.at<uint8_t>(iy, ix + dx);
            }
            intensity[k] = accum / 9.0;
        }

        // Z 방향 1D 미분(Gradient + 국소 분산 변화) 극대점 탐색으로 수면(Waterline) 위치 검출
        int best_k = n_samples / 2;
        double max_edge_response = -1.0;
        std::vector<double> edge_resp(n_samples, 0.0);

        for (int k = 4; k < n_samples - 4; ++k) {
            // 위쪽(선체+마크)과 아래쪽(해수면)의 평균 밝기 대비 계산
            double upper_mean = (intensity[k + 1] + intensity[k + 2] + intensity[k + 3] + intensity[k + 4]) * 0.25;
            double lower_mean = (intensity[k - 1] + intensity[k - 2] + intensity[k - 3] + intensity[k - 4]) * 0.25;
            double contrast = std::abs(upper_mean - lower_mean);
            edge_resp[k] = contrast;
            if (contrast > max_edge_response) {
                max_edge_response = contrast;
                best_k = k;
            }
        }

        // 3점 포물선 서브픽셀 보간으로 정밀 순간 흘수 Z*(t) 확정
        double sub_k = (double)best_k;
        if (best_k >= 5 && best_k < n_samples - 5) {
            double e0 = edge_resp[best_k - 1], e1 = edge_resp[best_k], e2 = edge_resp[best_k + 1];
            double denom = e0 - 2.0 * e1 + e2;
            if (std::abs(denom) > 1e-6) {
                sub_k += 0.5 * (e0 - e2) / denom;
            }
        }

        double dz = (z_high - z_low) / (double)(n_samples - 1);
        double instant_draft = z_low + sub_k * dz;

        z_time_series_.push_back(instant_draft);
        prev_gray_ = curr_gray.clone();

        // 7. FFI 출력 텔레메트리 패킷 기록
        out->status_code = 1;
        out->instant_draft_m = instant_draft;
        out->waterline_u = (float)spine_u_of_z.eval(instant_draft);
        out->waterline_v = (float)spine_v_of_z.eval(instant_draft);
        out->active_mark_count = (int32_t)tracks_.size();

        for (size_t i = 0; i < tracks_.size(); ++i) {
            out->mark_u[i] = tracks_[i].uv.x;
            out->mark_v[i] = tracks_[i].uv.y;
            out->mark_z[i] = (float)tracks_[i].z_meter;
        }

        out->spine_pts_count = 50;
        for (int s = 0; s < 50; ++s) {
            double zs = z_low + (z_high - z_low) * s / 49.0;
            out->spine_u[s] = (float)spine_u_of_z.eval(zs);
            out->spine_v[s] = (float)spine_v_of_z.eval(zs);
        }
        out->collected_frames = (int32_t)z_time_series_.size();
    }

    // ------------------------------------------------------------------------
    // [모듈 D] 파군 적응형 AWLS + 파저 앵커링 + Sainflou 물리 보정 솔버
    // ------------------------------------------------------------------------
    static double percentile_vec(std::vector<double> v, double p) {
        if (v.empty()) return 0.0;
        std::sort(v.begin(), v.end());
        double idx = (p / 100.0) * (v.size() - 1);
        size_t lo = (size_t)std::floor(idx);
        size_t hi = (size_t)std::ceil(idx);
        double frac = idx - lo;
        return v[lo] * (1.0 - frac) + v[hi] * frac;
    }

    void solve_subset_awls(const std::vector<double>& z_in, double& z_true_out,
                           double& z_app_out, double& sainflou_mm_out,
                           double& h_wave_out, double& t_wave_out, double& rmse_mm_out) {
        size_t n = z_in.size();
        if (n < 60) return;

        // 1. Hampel 스파이크 제거 및 이동평균 속도(z_dot)/가속도(z_ddot) 산출
        std::vector<double> z_clean = z_in;
        int hw = std::max(3, (int)(0.6 * fps_));
        for (size_t i = 0; i < n; ++i) {
            std::vector<double> win;
            for (int j = (int)i - hw; j <= (int)i + hw; ++j) {
                if (j >= 0 && j < (int)n) win.push_back(z_in[j]);
            }
            double med = percentile_vec(win, 50.0);
            for (auto& val : win) val = std::abs(val - med);
            double mad = percentile_vec(win, 50.0) + 1e-5;
            if (std::abs(z_in[i] - med) > 2.5 * 1.4826 * mad) z_clean[i] = med;
        }

        std::vector<double> z_smooth(n), z_dot(n, 0.0), z_ddot(n, 0.0);
        int sw = std::max(3, (int)(0.5 * fps_));
        for (size_t i = 0; i < n; ++i) {
            double sum = 0.0; int cnt = 0;
            for (int j = (int)i - sw; j <= (int)i + sw; ++j) {
                if (j >= 0 && j < (int)n) { sum += z_clean[j]; cnt++; }
            }
            z_smooth[i] = sum / cnt;
        }
        for (size_t i = 2; i < n - 2; ++i) {
            z_dot[i] = (z_smooth[i + 2] - z_smooth[i - 2]) * fps_ / 4.0;
            z_ddot[i] = (z_smooth[i + 2] - 2.0 * z_smooth[i] + z_smooth[i - 2]) * (fps_ * fps_) / 4.0;
        }

        // 2. 고해상도 이산 푸리에 스캔(0.05Hz ~ 0.25Hz)으로 우세 너울 주파수 f_dom 탐색
        double z_base = percentile_vec(z_smooth, 40.0);
        double best_f = 0.10, max_power = -1.0;
        for (int fi = 0; fi <= 200; ++fi) {
            double f = 0.045 + 0.205 * (fi / 200.0);
            double w = 2.0 * M_PI * f;
            double re = 0.0, im = 0.0;
            for (size_t i = 0; i < n; ++i) {
                double t = i / fps_;
                double val = z_smooth[i] - z_base;
                re += val * std::cos(w * t);
                im += val * std::sin(w * t);
            }
            double pwr = re * re + im * im;
            if (pwr > max_power) { max_power = pwr; best_f = f; }
        }
        double omega = 2.0 * M_PI * best_f;
        t_wave_out = 1.0 / best_f;

        // 3. 파군 시변 포락선 A_env(t) 추출 (슬라이딩 1주기 파저 깊이 기준)
        int cyc_win = std::max(15, (int)(t_wave_out * fps_ * 0.55));
        std::vector<double> a_env(n, 0.25);
        for (size_t i = 0; i < n; ++i) {
            std::vector<double> sl;
            for (int j = (int)i - cyc_win; j <= (int)i + cyc_win; ++j) {
                if (j >= 0 && j < (int)n) sl.push_back(z_smooth[j]);
            }
            double depth = std::max(0.06, z_base - percentile_vec(sl, 10.0));
            a_env[i] = depth;
        }

        // 4. 유체역학 4위상 분리 가중치(w_base) 산출: 파저 바닥 + 건조 상승 제로크로싱 앵커링
        std::vector<double> w_base(n, 1.0);
        for (size_t i = 0; i < n; ++i) {
            double eta = (z_smooth[i] - z_base) / a_env[i];
            bool is_dry_upcross = (eta > -0.25) && (eta <= 0.25) && (z_dot[i] > 0.0);
            double w_elev = (eta <= 0.0 || is_dry_upcross) ? 1.0 : std::exp(-3.2 * eta * eta);
            double w_drain = (z_dot[i] < -v_drain_ && eta > -0.45) ? 0.04 : 1.0;
            double w_trough = (eta <= -0.45 && z_ddot[i] > 0.0) ? 1.3 : 1.0;
            w_base[i] = std::clamp(w_elev * w_drain * w_trough, 0.02, 1.0);
        }

        // 5. IRLS 가중 선형 직교 기저 회귀 (Z_app, C = s*cos(phi), S = -s*sin(phi), alpha)
        std::vector<double> w_total = w_base;
        double z_app = z_base, scale_a = 1.0, phi = 0.0, alpha = 0.0;
        const double g = 9.80665;
        const double stokes_c = 1.6;
        double t_mid = (n - 1) / (2.0 * fps_);

        for (int iter = 0; iter < 5; ++iter) {
            // 2차 고조파 항을 뺀 잔여 신호에 대해 4x4 가중 정규방정식 풀이
            cv::Mat XT_W_X = cv::Mat::zeros(4, 4, CV_64F);
            cv::Mat XT_W_Y = cv::Mat::zeros(4, 1, CV_64F);

            for (size_t i = 0; i < n; ++i) {
                double tc = (i / fps_) - t_mid;
                double amp_i = scale_a * a_env[i];
                double a2_i = (iter == 0) ? 0.0 : stokes_c * (omega * omega / (2.0 * g)) * (amp_i * amp_i);
                double y_adj = z_clean[i] - a2_i * std::cos(2.0 * (omega * tc + phi));

                double xvec[4] = {1.0, a_env[i] * std::cos(omega * tc), a_env[i] * std::sin(omega * tc), tc};
                double wi = w_total[i];

                for (int r = 0; r < 4; ++r) {
                    XT_W_Y.at<double>(r, 0) += wi * xvec[r] * y_adj;
                    for (int c = 0; c < 4; ++c) {
                        XT_W_X.at<double>(r, c) += wi * xvec[r] * xvec[c];
                    }
                }
            }

            cv::Mat beta;
            if (cv::solve(XT_W_X, XT_W_Y, beta, cv::DECOMP_SVD)) {
                z_app = beta.at<double>(0, 0);
                double C_val = beta.at<double>(1, 0);
                double S_val = beta.at<double>(2, 0);
                alpha = beta.at<double>(3, 0);
                scale_a = std::clamp(std::hypot(C_val, S_val), 0.4, 1.8);
                phi = std::atan2(-S_val, C_val);
            }

            // 포락선 정규화 잔차 기반 비대칭 Bisquare 가중치 갱신
            std::vector<double> r_norm(n), abs_r(n);
            for (size_t i = 0; i < n; ++i) {
                double tc = (i / fps_) - t_mid;
                double amp_i = scale_a * a_env[i];
                double a2_i = stokes_c * (omega * omega / (2.0 * g)) * (amp_i * amp_i);
                double pred = z_app + amp_i * std::cos(omega * tc + phi)
                              + a2_i * std::cos(2.0 * (omega * tc + phi)) + alpha * tc;
                r_norm[i] = (z_clean[i] - pred) / a_env[i];
                abs_r[i] = std::abs(r_norm[i]);
            }
            double sigma_r = 1.4826 * percentile_vec(abs_r, 50.0) + 1e-4;
            for (size_t i = 0; i < n; ++i) {
                double sc = (r_norm[i] > 0.0) ? (1.6 * sigma_r) : (3.5 * sigma_r);
                double u = std::min(1.0, std::abs(r_norm[i]) / sc);
                double w_irls = std::pow(1.0 - u * u, 2);
                w_total[i] = std::clamp(w_base[i] * w_irls, 1e-4, 1.0);
            }
        }

        // 6. Sainflou 중복파(Clapotis) 및 Flare 복사응력 수위 상승분 차감
        double sum_a = 0.0, sum_a2 = 0.0, sum_sq_err = 0.0;
        int cnt_trough = 0;
        for (size_t i = 0; i < n; ++i) {
            double tc = (i / fps_) - t_mid;
            double amp_i = scale_a * a_env[i];
            sum_a += amp_i;
            sum_a2 += amp_i * amp_i;

            if (w_total[i] > 0.6) {
                double a2_i = stokes_c * (omega * omega / (2.0 * g)) * (amp_i * amp_i);
                double pred = z_app + amp_i * std::cos(omega * tc + phi)
                              + a2_i * std::cos(2.0 * (omega * tc + phi)) + alpha * tc;
                sum_sq_err += std::pow(z_clean[i] - pred, 2);
                cnt_trough++;
            }
        }

        double mean_a = sum_a / n;
        double mean_a2 = sum_a2 / n;
        double flare_factor = 1.0 + std::tan(flare_rad_) * (mean_a * omega * omega / g);
        double sainflou_m = flare_factor * (mean_a2 * omega * omega) / (2.0 * g);

        z_app_out = z_app;
        sainflou_mm_out = sainflou_m * 1000.0;
        z_true_out = z_app - sainflou_m;
        h_wave_out = 2.0 * mean_a;
        rmse_mm_out = (cnt_trough > 0) ? (std::sqrt(sum_sq_err / cnt_trough) * 1000.0) : 0.0;
    }

    void compute_final_report(FinalSurveyResult* res) {
        size_t n = z_time_series_.size();
        if (n < 90) {
            res->is_valid = 0;
            return;
        }

        double z_true = 0, z_app = 0, sain_mm = 0, h_w = 0, t_w = 10.0, rmse_mm = 0;
        solve_subset_awls(z_time_series_, z_true, z_app, sain_mm, h_w, t_w, rmse_mm);

        // 전반부 / 후반부 Split-Half 무결성 자가 검산
        size_t half_n = n / 2;
        std::vector<double> first_half(z_time_series_.begin(), z_time_series_.begin() + std::max<size_t>(60, half_n));
        std::vector<double> second_half(z_time_series_.end() - std::max<size_t>(60, half_n), z_time_series_.end());

        double z_h1 = z_true, z_h2 = z_true, dummy;
        if (first_half.size() >= 60 && second_half.size() >= 60) {
            solve_subset_awls(first_half, z_h1, dummy, dummy, dummy, dummy, dummy);
            solve_subset_awls(second_half, z_h2, dummy, dummy, dummy, dummy, dummy);
        }

        double split_diff_mm = std::abs(z_h1 - z_h2) * 1000.0;
        double simple_mean = std::accumulate(z_time_series_.begin(), z_time_series_.end(), 0.0) / n;
        double cycles = (n / fps_) / t_w;

        res->is_valid = 1;
        res->true_draft_m = z_true;
        res->apparent_mean_m = z_app;
        res->simple_mean_m = simple_mean;
        res->sainflou_deduct_mm = sain_mm;
        res->swell_height_m = h_w;
        res->swell_period_s = t_w;
        res->observed_cycles = cycles;
        res->trough_rmse_mm = rmse_mm;
        res->split_half_diff_mm = split_diff_mm;
        res->reliability_Pass = (cycles >= 3.0 && split_diff_mm <= 12.0 && rmse_mm <= 18.0) ? 1 : 0;
    }
};

// ============================================================================
// 5. C-ABI FFI Export 함수 (Dart FFI 바인딩 엔드포인트)
// ============================================================================
FFI_EXPORT void* draft_engine_create(double fps, double flare_angle_deg) {
    return new NativeDraftEngine(fps, flare_angle_deg);
}

FFI_EXPORT void draft_engine_destroy(void* handle) {
    if (handle) delete static_cast<NativeDraftEngine*>(handle);
}

FFI_EXPORT int32_t draft_engine_bootstrap_seeds(
    void* handle, const uint8_t* y_plane, int32_t width, int32_t height, int32_t stride,
    int32_t seed_count, const float* seed_u, const float* seed_v, const float* seed_z
) {
    if (!handle || !y_plane) return 0;
    cv::Mat gray(height, width, CV_8UC1, const_cast<uint8_t*>(y_plane), stride);
    return static_cast<NativeDraftEngine*>(handle)->bootstrap_from_seeds(
        gray, seed_count, seed_u, seed_v, seed_z
    ) ? 1 : 0;
}

FFI_EXPORT void draft_engine_process_frame(
    void* handle, const uint8_t* y_plane, int32_t width, int32_t height, int32_t stride,
    float gx, float gy, float gz, TelemetryOutput* out_telemetry
) {
    if (!handle || !y_plane || !out_telemetry) return;
    cv::Mat gray(height, width, CV_8UC1, const_cast<uint8_t*>(y_plane), stride);
    static_cast<NativeDraftEngine*>(handle)->process_frame(gray, gx, gy, gz, out_telemetry);
}

FFI_EXPORT void draft_engine_solve_final(void* handle, FinalSurveyResult* out_result) {
    if (!handle || !out_result) return;
    static_cast<NativeDraftEngine*>(handle)->compute_final_report(out_result);
}
