#pragma once

#include <cstddef>

/*
 * Migration map:
 *   Estimator::shouldSkipVisualFrame -> VisualHealthMonitor::shouldSkipVisualFrame
 *   Estimator::updateVisualHealth    -> VisualHealthMonitor::classify
 *   Estimator::enterBlind/exitBlind  -> still live in Estimator for blind-anchor bookkeeping
 *
 * This helper is intentionally stateless for now. It gives the visual-health
 * heuristics one narrow home before we move the state machine out of Estimator.
 */

enum VisualState
{
    VISUAL_HEALTHY = 0,
    VISUAL_DEGRADED = 1,
    VISUAL_BLIND = 2
};

struct VisualHealthSnapshot
{
    int image_points = 0;
    int tracked_points = 0;
    int prev_points = 0;
    int tracked_after_lk = 0;
    int tracked_after_ransac = 0;
    int total_points = 0;
    double lk_keep_ratio = 1.0;
    double mean_track_eigen = -1.0;
    double mean_pixel_flow = 0.0;
    double coverage_ratio = 0.0;
    bool low_tracking_quality = false;
    bool weak_texture = false;
    bool poor_distribution = false;
    bool ransac_rejected = false;
    int visual_track_num = 0;
    double visual_parallax = 0.0;
};

class VisualHealthMonitor
{
  public:
    bool shouldSkipVisualFrame(const VisualHealthSnapshot &snapshot,
                               bool blind_enable,
                               bool use_imu,
                               bool solver_non_linear,
                               VisualState current_state,
                               bool blind_active) const;

    VisualState classify(const VisualHealthSnapshot &snapshot,
                         bool blind_enable,
                         bool solver_non_linear,
                         VisualState current_state,
                         bool blind_active) const;
};
