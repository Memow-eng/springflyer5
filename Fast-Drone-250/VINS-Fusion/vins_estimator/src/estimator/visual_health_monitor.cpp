#include "visual_health_monitor.h"

#include <algorithm>

#include "../estimator/parameters.h"

bool VisualHealthMonitor::shouldSkipVisualFrame(const VisualHealthSnapshot &snapshot,
                                                bool blind_enable,
                                                bool use_imu,
                                                bool solver_non_linear,
                                                VisualState current_state,
                                                bool blind_active) const
{
    if (!blind_enable || !use_imu || !solver_non_linear)
        return false;
    const int pts = snapshot.visual_track_num > 0 ? snapshot.visual_track_num : snapshot.image_points;
    const int detectable = snapshot.total_points > 0 ? snapshot.total_points : snapshot.image_points;
    const bool truly_blind = detectable < BLIND_PTS_BLIND;
    const bool hold = (current_state == VISUAL_BLIND) && blind_active &&
                      (pts < BLIND_PTS_OK);
    return truly_blind || hold;
}

VisualState VisualHealthMonitor::classify(const VisualHealthSnapshot &snapshot,
                                          bool blind_enable,
                                          bool solver_non_linear,
                                          VisualState current_state,
                                          bool blind_active) const
{
    if (!blind_enable || !solver_non_linear)
        return current_state;

    const int pts = snapshot.visual_track_num > 0 ? snapshot.visual_track_num : snapshot.image_points;
    // "blind" is about how much structure the camera can SEE, not how many old
    // tracks survived the last frame. Fast motion drops surviving tracks while
    // the detector still returns plenty of well-distributed points, so the
    // blindness decision keys off detectable points, not visual_track_num.
    const int detectable = snapshot.total_points > 0 ? snapshot.total_points : snapshot.image_points;
    const double flow = snapshot.mean_pixel_flow;
    const double eig = snapshot.mean_track_eigen;

    const bool dynamic = (detectable >= BLIND_PTS_OK) && (flow > BLIND_FLOW_DYN);
    const int ransac_survivors = snapshot.tracked_after_ransac;
    // High-speed re-detection: fast motion, but the detector still returns
    // plenty of well-distributed, RANSAC-consistent points. Quality statistics
    // are dominated by track AGE here and must NOT drive a blind decision.
    const bool fast_redetect =
        (flow > BLIND_FLOW_DYN) &&
        (detectable >= BLIND_PTS_OK) &&
        (snapshot.coverage_ratio >= 0.6) &&
        (ransac_survivors >= BLIND_PTS_OK / 2);
    // Genuine loss of usable structure (track-age independent geometry only).
    const bool geometry_collapsed =
        (detectable < BLIND_PTS_MIN) ||
        (snapshot.coverage_ratio < 0.35) ||
        ((snapshot.prev_points >= BLIND_PTS_OK) && (ransac_survivors < BLIND_PTS_BLIND));
    const bool low_flow_static =
        (pts >= LOW_FLOW_ZUPT_MIN_TRACKS) &&
        (snapshot.lk_keep_ratio > 0.80) &&
        (flow >= 0.0 && flow < LOW_FLOW_ZUPT_FLOW);
    const bool cant_detect = detectable < BLIND_PTS_MIN;
    const bool weak_texture_static =
        (eig >= 0.0 && eig < FRONTEND_QUALITY_MIN_EIGEN) &&
        (flow < BLIND_FLOW_STATIC);
    const bool quality_collapsed =
        ((snapshot.quality_median < QUALITY_DEGRADED_MEDIAN) &&
         (snapshot.quality_bad_ratio > QUALITY_DEGRADED_BAD_RATIO)) ||
        ((snapshot.high_quality_long_ratio < QUALITY_DEGRADED_HQ_LONG_RATIO) &&
         (snapshot.new_feature_ratio > QUALITY_DEGRADED_NEW_RATIO));
    const bool track_reset =
        (snapshot.new_feature_ratio > std::max(0.65, QUALITY_DEGRADED_NEW_RATIO)) &&
        (snapshot.high_quality_long_ratio < QUALITY_DEGRADED_HQ_LONG_RATIO);
    const bool quality_blind =
        !fast_redetect &&
        track_reset &&
        (pts < BLIND_PTS_BLIND / 2) &&
        (snapshot.quality_bad_ratio > std::max(0.50, QUALITY_DEGRADED_BAD_RATIO) ||
         snapshot.quality_median < 0.5 * QUALITY_DEGRADED_MEDIAN);
    const bool genuinely_degraded =
        cant_detect || weak_texture_static || snapshot.poor_distribution || quality_collapsed;

    VisualState target;
    if (detectable < BLIND_PTS_BLIND || geometry_collapsed || quality_blind)
        target = VISUAL_BLIND;
    else if (genuinely_degraded && !dynamic)
        target = VISUAL_DEGRADED;
    else
        target = VISUAL_HEALTHY;

    if (current_state == VISUAL_BLIND && target != VISUAL_BLIND)
    {
        // Recovery must NOT require a low new-feature ratio: during sustained
        // fast motion new_feature_ratio stays high, so that clause would lock
        // the estimator in BLIND for the whole dynamic segment. A clearly
        // healthy fast-re-detection frame recovers immediately; otherwise fall
        // back to the parallax/quality/coverage test.
        const bool recovery_quality =
            snapshot.quality_median >= std::max(0.70, QUALITY_DEGRADED_MEDIAN) &&
            snapshot.visual_parallax >= std::max(2.0, BLIND_PARALLAX_THRESHOLD) &&
            snapshot.coverage_ratio >= 0.5;
        const bool recovered =
            fast_redetect ||
            ((detectable >= BLIND_PTS_OK) && !genuinely_degraded &&
             recovery_quality && !low_flow_static);
        if (!recovered)
            target = VISUAL_BLIND;
    }
    return target;
}
