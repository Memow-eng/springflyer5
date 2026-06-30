#pragma once

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
    double brightness_mean = -1.0;
    double dark_ratio = 0.0;
    double saturated_ratio = 0.0;
    double contrast_std = 0.0;
    double blur_score = 0.0;
    double photometric_health = 1.0;
    bool low_tracking_quality = false;
    bool weak_texture = false;
    bool poor_distribution = false;
    bool ransac_rejected = false;
    bool low_parallax = false;
    int visual_track_num = 0;
    double visual_parallax = 0.0;
    double quality_median = 1.0;
    double quality_bad_ratio = 0.0;
    double high_quality_long_ratio = 1.0;
    double new_feature_ratio = 0.0;
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
