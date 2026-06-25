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

    const int image_points = snapshot.image_points;
    const int tracked_points = snapshot.prev_points > 0
                                   ? snapshot.tracked_after_lk
                                   : image_points;
    const bool very_few_tracks = tracked_points <= BLIND_ENTER_TRACK_NUM ||
                                 image_points <= BLIND_ENTER_TRACK_NUM;
    const bool severe_lk_loss =
        snapshot.prev_points >= FRONTEND_QUALITY_MIN_TRACKED &&
        snapshot.lk_keep_ratio < 0.2 &&
        tracked_points <= BLIND_DEGRADED_TRACK_NUM;
    const bool severe_texture =
        snapshot.mean_track_eigen >= 0.0 &&
        snapshot.mean_track_eigen < FRONTEND_QUALITY_MIN_EIGEN;
    const bool weak_texture_with_few_tracks =
        snapshot.weak_texture && tracked_points <= BLIND_DEGRADED_TRACK_NUM;
    const bool hold_blind_until_recovered =
        current_state == VISUAL_BLIND && blind_active &&
        (tracked_points < BLIND_EXIT_TRACK_NUM ||
         snapshot.low_tracking_quality ||
         snapshot.weak_texture ||
         snapshot.poor_distribution);

    return very_few_tracks || severe_lk_loss || severe_texture ||
           weak_texture_with_few_tracks || hold_blind_until_recovered;
}

VisualState VisualHealthMonitor::classify(const VisualHealthSnapshot &snapshot,
                                          bool blind_enable,
                                          bool solver_non_linear,
                                          VisualState current_state,
                                          bool blind_active) const
{
    if (!blind_enable || !solver_non_linear)
        return current_state;

    const bool frontend_low_tracking = snapshot.low_tracking_quality;
    const bool frontend_weak_texture = snapshot.weak_texture;
    const bool frontend_poor_distribution = snapshot.poor_distribution;
    const int frontend_track_num = snapshot.prev_points > 0
                                       ? snapshot.tracked_after_lk
                                       : snapshot.image_points;
    const bool high_motion_with_features =
        snapshot.mean_pixel_flow > 8.0 &&
        snapshot.image_points > BLIND_EXIT_TRACK_NUM &&
        frontend_track_num > BLIND_ENTER_TRACK_NUM;
    const bool frontend_severe_texture =
        snapshot.mean_track_eigen >= 0.0 &&
        snapshot.mean_track_eigen < FRONTEND_QUALITY_MIN_EIGEN;
    const bool frontend_ransac_bad =
        snapshot.prev_points >= FRONTEND_QUALITY_MIN_TRACKED &&
        snapshot.tracked_after_ransac <= BLIND_DEGRADED_TRACK_NUM;
    const bool frontend_degraded = frontend_low_tracking || frontend_weak_texture ||
                                   frontend_poor_distribution || frontend_ransac_bad;
    const bool frontend_blind = snapshot.image_points <= BLIND_ENTER_TRACK_NUM ||
                                frontend_track_num <= BLIND_ENTER_TRACK_NUM ||
                                frontend_severe_texture ||
                                (!high_motion_with_features &&
                                 ((frontend_weak_texture &&
                                   frontend_track_num <= BLIND_DEGRADED_TRACK_NUM) ||
                                  (frontend_low_tracking &&
                                   snapshot.lk_keep_ratio < 0.2 &&
                                   frontend_track_num <= BLIND_DEGRADED_TRACK_NUM)));
    const bool enough_tracks_to_exit = snapshot.visual_track_num >= BLIND_EXIT_TRACK_NUM;
    const bool enough_parallax_to_exit = snapshot.visual_parallax >= BLIND_PARALLAX_THRESHOLD;
    const bool visual_recovered = enough_tracks_to_exit && enough_parallax_to_exit &&
                                  !frontend_degraded;
    const bool has_track_support =
        snapshot.visual_track_num >= std::max(BLIND_EXIT_TRACK_NUM, BLIND_DEGRADED_TRACK_NUM + 10) &&
        snapshot.tracked_points >= std::max(BLIND_EXIT_TRACK_NUM, BLIND_DEGRADED_TRACK_NUM + 10) &&
        snapshot.lk_keep_ratio >= std::max(0.70, FRONTEND_MIN_LK_KEEP_RATIO);
    const bool low_parallax_only =
        snapshot.visual_parallax < BLIND_PARALLAX_THRESHOLD &&
        has_track_support &&
        !frontend_degraded &&
        !frontend_blind;

    VisualState next_state = current_state;
    if (current_state == VISUAL_BLIND)
    {
        if (visual_recovered)
            next_state = VISUAL_HEALTHY;
        else
            next_state = VISUAL_BLIND;
    }
    else
    {
        if (low_parallax_only)
            return VISUAL_HEALTHY;
        if ((!high_motion_with_features && snapshot.visual_track_num <= BLIND_ENTER_TRACK_NUM) || frontend_blind)
            next_state = VISUAL_BLIND;
        else if (snapshot.visual_track_num <= BLIND_DEGRADED_TRACK_NUM || frontend_degraded)
            next_state = VISUAL_DEGRADED;
        else if (visual_recovered)
            next_state = VISUAL_HEALTHY;
        else
            next_state = VISUAL_DEGRADED;
    }

    return next_state;
}
