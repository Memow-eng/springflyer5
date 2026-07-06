/*******************************************************
 * Copyright (C) 2019, Aerial Robotics Group, Hong Kong University of Science and Technology
 * 
 * This file is part of VINS.
 * 
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *
 * Author: Qin Tong (qintonguav@gmail.com)
 *******************************************************/

#include "feature_tracker.h"
#include <algorithm>
#include <fstream>
#include <iomanip>

namespace
{
int gridIdForPoint(const cv::Point2f &p, int width, int height, int grid_cols, int grid_rows)
{
    const int safe_width = std::max(1, width);
    const int safe_height = std::max(1, height);
    const int safe_cols = std::max(1, grid_cols);
    const int safe_rows = std::max(1, grid_rows);
    int cx = static_cast<int>(p.x * safe_cols / safe_width);
    int cy = static_cast<int>(p.y * safe_rows / safe_height);
    cx = std::max(0, std::min(cx, safe_cols - 1));
    cy = std::max(0, std::min(cy, safe_rows - 1));
    return cy * safe_cols + cx;
}

struct FrontendCoverageStats
{
    int grid_cols = 0;
    int grid_rows = 0;
    int total_cells = 0;
    int tracked_count = 0;
    int new_count = 0;
    int total_count = 0;
    int occupied_cells = 0;
    int tracked_occupied_cells = 0;
    int new_occupied_cells = 0;
    int bottom_total_count = 0;
    int bottom_tracked_count = 0;
    int bottom_new_count = 0;
    int low_motion_total_count = 0;
    int low_motion_tracked_count = 0;
    int low_motion_new_count = 0;
    int max_cell_count = 0;
    int min_nonzero_cell_count = 0;
    double coverage_ratio = 0.0;
    double tracked_coverage_ratio = 0.0;
    double new_coverage_ratio = 0.0;
    double bottom_total_ratio = 0.0;
    double bottom_tracked_ratio = 0.0;
    double bottom_new_ratio = 0.0;
    double low_motion_total_ratio = 0.0;
    double low_motion_tracked_ratio = 0.0;
    double low_motion_new_ratio = 0.0;
};

FrontendCoverageStats computeCoverageStats(const std::vector<cv::Point2f> &tracked_pts,
                                           const std::vector<cv::Point2f> &new_pts,
                                           const std::vector<int> &tracked_ids,
                                           const std::map<int, cv::Point2f> &prev_pts_by_id,
                                           int width,
                                           int height,
                                           int configured_cols,
                                           int configured_rows)
{
    FrontendCoverageStats stats;
    stats.grid_cols = std::max(1, std::min(configured_cols, std::max(1, width)));
    stats.grid_rows = std::max(1, std::min(configured_rows, std::max(1, height)));
    stats.total_cells = stats.grid_cols * stats.grid_rows;
    stats.tracked_count = static_cast<int>(tracked_pts.size());
    stats.new_count = static_cast<int>(new_pts.size());
    stats.total_count = stats.tracked_count + stats.new_count;

    std::vector<int> total_cells(stats.total_cells, 0);
    std::vector<int> tracked_cells(stats.total_cells, 0);
    std::vector<int> new_cells(stats.total_cells, 0);

    const double bottom_y = 0.70 * std::max(1, height);
    const double low_motion_px = std::max(1.0, 0.25 * MIN_DIST);

    for (size_t i = 0; i < tracked_pts.size(); ++i)
    {
        const cv::Point2f &p = tracked_pts[i];
        const int gid = gridIdForPoint(p, width, height, stats.grid_cols, stats.grid_rows);
        total_cells[gid]++;
        tracked_cells[gid]++;
        if (p.y >= bottom_y)
            stats.bottom_tracked_count++;
        if (i < tracked_ids.size())
        {
            const auto prev_it = prev_pts_by_id.find(tracked_ids[i]);
            if (prev_it != prev_pts_by_id.end() && cv::norm(p - prev_it->second) <= low_motion_px)
                stats.low_motion_tracked_count++;
        }
    }
    for (const auto &p : new_pts)
    {
        const int gid = gridIdForPoint(p, width, height, stats.grid_cols, stats.grid_rows);
        total_cells[gid]++;
        new_cells[gid]++;
        if (p.y >= bottom_y)
            stats.bottom_new_count++;
    }

    stats.bottom_total_count = stats.bottom_tracked_count + stats.bottom_new_count;
    stats.low_motion_new_count = stats.new_count;
    stats.low_motion_total_count = stats.low_motion_tracked_count + stats.low_motion_new_count;

    stats.min_nonzero_cell_count = stats.total_count > 0 ? stats.total_count : 0;
    for (int i = 0; i < stats.total_cells; ++i)
    {
        if (total_cells[i] > 0)
        {
            stats.occupied_cells++;
            stats.max_cell_count = std::max(stats.max_cell_count, total_cells[i]);
            stats.min_nonzero_cell_count = std::min(stats.min_nonzero_cell_count, total_cells[i]);
        }
        if (tracked_cells[i] > 0)
            stats.tracked_occupied_cells++;
        if (new_cells[i] > 0)
            stats.new_occupied_cells++;
    }

    if (stats.total_cells > 0)
    {
        stats.coverage_ratio = static_cast<double>(stats.occupied_cells) / stats.total_cells;
        stats.tracked_coverage_ratio = static_cast<double>(stats.tracked_occupied_cells) / stats.total_cells;
        stats.new_coverage_ratio = static_cast<double>(stats.new_occupied_cells) / stats.total_cells;
    }
    if (stats.total_count > 0)
    {
        stats.bottom_total_ratio = static_cast<double>(stats.bottom_total_count) / stats.total_count;
        stats.low_motion_total_ratio = static_cast<double>(stats.low_motion_total_count) / stats.total_count;
    }
    if (stats.tracked_count > 0)
    {
        stats.bottom_tracked_ratio = static_cast<double>(stats.bottom_tracked_count) / stats.tracked_count;
        stats.low_motion_tracked_ratio = static_cast<double>(stats.low_motion_tracked_count) / stats.tracked_count;
    }
    if (stats.new_count > 0)
    {
        stats.bottom_new_ratio = static_cast<double>(stats.bottom_new_count) / stats.new_count;
        stats.low_motion_new_ratio = static_cast<double>(stats.low_motion_new_count) / stats.new_count;
    }

    return stats;
}

void writeFrontendCoverageSidecar(double timestamp,
                                  const FrontendCoverageStats &stats,
                                  bool grid_feature_enabled)
{
    if (OUTPUT_FOLDER.empty())
        return;

    const std::string path = OUTPUT_FOLDER + "/frontend_coverage_stats.csv";
    static bool header_written = false;
    std::ofstream out(path.c_str(), std::ios::app);
    if (!out.is_open())
        return;

    if (!header_written)
    {
        out << "timestamp,grid_feature_enabled,grid_cols,grid_rows,total_cells,total_count,tracked_count,new_count,occupied_cells,coverage_ratio,tracked_occupied_cells,tracked_coverage_ratio,new_occupied_cells,new_coverage_ratio,bottom_total_count,bottom_total_ratio,bottom_tracked_count,bottom_tracked_ratio,bottom_new_count,bottom_new_ratio,low_motion_total_count,low_motion_total_ratio,low_motion_tracked_count,low_motion_tracked_ratio,low_motion_new_count,low_motion_new_ratio,min_nonzero_cell_count,max_cell_count\n";
        header_written = true;
    }

    out << std::fixed << std::setprecision(9)
        << timestamp << ','
        << (grid_feature_enabled ? 1 : 0) << ','
        << stats.grid_cols << ','
        << stats.grid_rows << ','
        << stats.total_cells << ','
        << stats.total_count << ','
        << stats.tracked_count << ','
        << stats.new_count << ','
        << stats.occupied_cells << ','
        << stats.coverage_ratio << ','
        << stats.tracked_occupied_cells << ','
        << stats.tracked_coverage_ratio << ','
        << stats.new_occupied_cells << ','
        << stats.new_coverage_ratio << ','
        << stats.bottom_total_count << ','
        << stats.bottom_total_ratio << ','
        << stats.bottom_tracked_count << ','
        << stats.bottom_tracked_ratio << ','
        << stats.bottom_new_count << ','
        << stats.bottom_new_ratio << ','
        << stats.low_motion_total_count << ','
        << stats.low_motion_total_ratio << ','
        << stats.low_motion_tracked_count << ','
        << stats.low_motion_tracked_ratio << ','
        << stats.low_motion_new_count << ','
        << stats.low_motion_new_ratio << ','
        << stats.min_nonzero_cell_count << ','
        << stats.max_cell_count << '\n';
}
} // namespace

bool FeatureTracker::inBorder(const cv::Point2f &pt)
{
    const int BORDER_SIZE = 1;
    int img_x = cvRound(pt.x);
    int img_y = cvRound(pt.y);
    return BORDER_SIZE <= img_x && img_x < col - BORDER_SIZE && BORDER_SIZE <= img_y && img_y < row - BORDER_SIZE;
}

double distance(cv::Point2f pt1, cv::Point2f pt2)
{
    //printf("pt1: %f %f pt2: %f %f\n", pt1.x, pt1.y, pt2.x, pt2.y);
    double dx = pt1.x - pt2.x;
    double dy = pt1.y - pt2.y;
    return sqrt(dx * dx + dy * dy);
}

void reduceVector(vector<cv::Point2f> &v, vector<uchar> status)
{
    int j = 0;
    for (int i = 0; i < int(v.size()); i++)
        if (status[i])
            v[j++] = v[i];
    v.resize(j);
}

void reduceVector(vector<int> &v, vector<uchar> status)
{
    int j = 0;
    for (int i = 0; i < int(v.size()); i++)
        if (status[i])
            v[j++] = v[i];
    v.resize(j);
}

FeatureTracker::FeatureTracker()
{
    stereo_cam = 0;
    n_id = 0;
    hasPrediction = false;
    last_total_feature_count_ = 0;
    last_tracked_feature_count_ = 0;
    last_new_feature_count_ = 0;
    last_grid_coverage_ratio_ = 0.0;
}

void FeatureTracker::setMask()
{
    mask = cv::Mat(row, col, CV_8UC1, cv::Scalar(255));

    // prefer to keep features that are tracked for long time
    vector<pair<int, pair<cv::Point2f, int>>> cnt_pts_id;

    for (unsigned int i = 0; i < cur_pts.size(); i++)
        cnt_pts_id.push_back(make_pair(track_cnt[i], make_pair(cur_pts[i], ids[i])));

    sort(cnt_pts_id.begin(), cnt_pts_id.end(), [](const pair<int, pair<cv::Point2f, int>> &a, const pair<int, pair<cv::Point2f, int>> &b)
         {
            return a.first > b.first;
         });

    cur_pts.clear();
    ids.clear();
    track_cnt.clear();

    for (auto &it : cnt_pts_id)
    {
        if (mask.at<uchar>(it.second.first) == 255)
        {
            cur_pts.push_back(it.second.first);
            ids.push_back(it.second.second);
            track_cnt.push_back(it.first);
            cv::circle(mask, it.second.first, MIN_DIST, 0, -1);
        }
    }
}

void FeatureTracker::extractGridFeatures()
{
    n_pts.clear();
    const int total_need = MAX_CNT - static_cast<int>(cur_pts.size());
    if (total_need <= 0 || cur_img.empty() || mask.empty())
        return;

    if (!FRONTEND_GRID_FEATURE_ENABLE)
    {
        cv::goodFeaturesToTrack(cur_img, n_pts, total_need, 0.01, MIN_DIST, mask);
        return;
    }

    const int grid_cols = std::max(1, std::min(FRONTEND_GRID_COLS, std::max(1, col)));
    const int grid_rows = std::max(1, std::min(FRONTEND_GRID_ROWS, std::max(1, row)));
    const int max_per_cell = std::max(1, FRONTEND_GRID_MAX_PER_CELL);
    const double quality = FRONTEND_GRID_QUALITY > 0.0 ? FRONTEND_GRID_QUALITY : 0.01;
    const int n_cells = grid_cols * grid_rows;

    if (static_cast<int>(grid_cnt_.size()) != n_cells)
        grid_cnt_.assign(n_cells, 0);
    else
        std::fill(grid_cnt_.begin(), grid_cnt_.end(), 0);

    // Count already-tracked points per cell so cells that setMask already
    // populated with long-lived features are not topped up with new corners.
    for (const auto &p : cur_pts)
    {
        if (!inBorder(p))
            continue;
        const int gid = gridIdForPoint(p, col, row, grid_cols, grid_rows);
        grid_cnt_[gid]++;
    }

    // One global detection over the whole image. goodFeaturesToTrack returns
    // corners sorted by Shi-Tomasi response (strongest first) and enforces
    // MIN_DIST across the entire frame via the shared mask, so there is no
    // per-ROI boundary clumping and strong-texture regions are preferred.
    // We over-request (2x budget) so that after per-cell capping we still have
    // enough candidates to fill the frame, without lowering the quality floor.
    cell_pts_.clear();
    const int request = std::min(4 * MAX_CNT, std::max(total_need, 2 * MAX_CNT));
    cv::goodFeaturesToTrack(cur_img, cell_pts_, request, quality, MIN_DIST, mask, 3, false, 0.04);

    // Absolute Shi-Tomasi response floor: reject globally-weak corners outright
    // instead of forcing every cell to fill. When FRONTEND_MIN_EIG <= 0 this is
    // a no-op and behaviour is response-ordered budget only.
    std::vector<float> eig;
    if (FRONTEND_MIN_EIG > 0.0 && !cell_pts_.empty())
        cv::cornerMinEigenVal(cur_img, min_eig_map_, 3);

    n_pts.reserve(total_need);
    for (const auto &p : cell_pts_)
    {
        if (static_cast<int>(n_pts.size()) >= total_need)
            break;
        if (!inBorder(p) || mask.at<uchar>(p) == 0)
            continue;
        if (FRONTEND_MIN_EIG > 0.0 &&
            min_eig_map_.at<float>(cvRound(p.y), cvRound(p.x)) < FRONTEND_MIN_EIG)
            continue;

        const int gid = gridIdForPoint(p, col, row, grid_cols, grid_rows);
        if (grid_cnt_[gid] >= max_per_cell)
            continue;

        n_pts.push_back(p);
        grid_cnt_[gid]++;
        cv::circle(mask, p, MIN_DIST, 0, -1);
    }
}

double FeatureTracker::distance(cv::Point2f &pt1, cv::Point2f &pt2)
{
    //printf("pt1: %f %f pt2: %f %f\n", pt1.x, pt1.y, pt2.x, pt2.y);
    double dx = pt1.x - pt2.x;
    double dy = pt1.y - pt2.y;
    return sqrt(dx * dx + dy * dy);
}

map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>> FeatureTracker::trackImage(double _cur_time, const cv::Mat &_img, const cv::Mat &_img1)
{
    TicToc t_r;
    cur_time = _cur_time;
    cur_img = _img;
    row = cur_img.rows;
    col = cur_img.cols;
    cv::Mat rightImg = _img1;
    /*
    {
        cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(3.0, cv::Size(8, 8));
        clahe->apply(cur_img, cur_img);
        if(!rightImg.empty())
            clahe->apply(rightImg, rightImg);
    }
    */
    cur_pts.clear();

    if (prev_pts.size() > 0)
    {
        TicToc t_o;
        vector<uchar> status;
        vector<float> err;
        if(hasPrediction)
        {
            cur_pts = predict_pts;
            cv::calcOpticalFlowPyrLK(prev_img, cur_img, prev_pts, cur_pts, status, err, cv::Size(21, 21), 1, 
            cv::TermCriteria(cv::TermCriteria::COUNT+cv::TermCriteria::EPS, 30, 0.01), cv::OPTFLOW_USE_INITIAL_FLOW);
            
            int succ_num = 0;
            for (size_t i = 0; i < status.size(); i++)
            {
                if (status[i])
                    succ_num++;
            }
            if (succ_num < 10)
               cv::calcOpticalFlowPyrLK(prev_img, cur_img, prev_pts, cur_pts, status, err, cv::Size(21, 21), 3);
        }
        else
            cv::calcOpticalFlowPyrLK(prev_img, cur_img, prev_pts, cur_pts, status, err, cv::Size(21, 21), 3);
        // reverse check
        if(FLOW_BACK)
        {
            vector<uchar> reverse_status;
            vector<cv::Point2f> reverse_pts = prev_pts;
            cv::calcOpticalFlowPyrLK(cur_img, prev_img, cur_pts, reverse_pts, reverse_status, err, cv::Size(21, 21), 1, 
            cv::TermCriteria(cv::TermCriteria::COUNT+cv::TermCriteria::EPS, 30, 0.01), cv::OPTFLOW_USE_INITIAL_FLOW);
            //cv::calcOpticalFlowPyrLK(cur_img, prev_img, cur_pts, reverse_pts, reverse_status, err, cv::Size(21, 21), 3); 
            for(size_t i = 0; i < status.size(); i++)
            {
                double fb_error = -1.0;
                if(status[i] && reverse_status[i])
                    fb_error = distance(prev_pts[i], reverse_pts[i]);
                if(status[i] && reverse_status[i] && fb_error <= FRONTEND_FB_THRESHOLD)
                {
                    status[i] = 1;
                }
                else
                    status[i] = 0;
            }
        }
        
        for (int i = 0; i < int(cur_pts.size()); i++)
            if (status[i] && !inBorder(cur_pts[i]))
                status[i] = 0;
        reduceVector(prev_pts, status);
        reduceVector(cur_pts, status);
        reduceVector(ids, status);
        reduceVector(track_cnt, status);
        ROS_DEBUG("temporal optical flow costs: %fms", t_o.toc());
        //printf("track cnt %d\n", (int)ids.size());
    }

    for (auto &n : track_cnt)
        n++;

    if (1)
    {
        rejectWithF();
        ROS_DEBUG("set mask begins");
        TicToc t_m;
        setMask();
        ROS_DEBUG("set mask costs %fms", t_m.toc());

        ROS_DEBUG("detect feature begins");
        TicToc t_t;
        int n_max_cnt = MAX_CNT - static_cast<int>(cur_pts.size());
        if (n_max_cnt > 0)
        {
            if(mask.empty())
                cout << "mask is empty " << endl;
            if (mask.type() != CV_8UC1)
                cout << "mask type wrong " << endl;
            extractGridFeatures();
        }
        else
            n_pts.clear();
        ROS_DEBUG("detect feature costs: %f ms", t_t.toc());

        const std::vector<cv::Point2f> tracked_pts_before_new = cur_pts;
        const std::vector<int> tracked_ids_before_new = ids;
        const std::map<int, cv::Point2f> prev_pts_by_id_for_coverage = prevLeftPtsMap;
        const int tracked_count_before_new = static_cast<int>(cur_pts.size());
        const int new_count_added = static_cast<int>(n_pts.size());

        for (auto &p : n_pts)
        {
            cur_pts.push_back(p);
            ids.push_back(n_id++);
            track_cnt.push_back(1);
        }

        FrontendCoverageStats coverage_stats = computeCoverageStats(tracked_pts_before_new,
                                                                    n_pts,
                                                                    tracked_ids_before_new,
                                                                    prev_pts_by_id_for_coverage,
                                                                    col,
                                                                    row,
                                                                    FRONTEND_GRID_COLS,
                                                                    FRONTEND_GRID_ROWS);
        last_total_feature_count_ = static_cast<int>(cur_pts.size());
        last_tracked_feature_count_ = tracked_count_before_new;
        last_new_feature_count_ = new_count_added;
        last_grid_coverage_ratio_ = coverage_stats.coverage_ratio;
        writeFrontendCoverageSidecar(cur_time, coverage_stats, FRONTEND_GRID_FEATURE_ENABLE != 0);
        ROS_INFO_STREAM_THROTTLE(1.0, "FRONTEND_COVERAGE total=" << last_total_feature_count_
                                 << " tracked=" << last_tracked_feature_count_
                                 << " new=" << last_new_feature_count_
                                 << " coverage=" << std::fixed << std::setprecision(3)
                                 << last_grid_coverage_ratio_);
        //printf("feature cnt after add %d\n", (int)ids.size());
    }

    cur_un_pts = undistortedPts(cur_pts, m_camera[0]);
    pts_velocity = ptsVelocity(ids, cur_un_pts, cur_un_pts_map, prev_un_pts_map);

    if(!_img1.empty() && stereo_cam)
    {
        ids_right.clear();
        cur_right_pts.clear();
        cur_un_right_pts.clear();
        right_pts_velocity.clear();
        cur_un_right_pts_map.clear();
        if(!cur_pts.empty())
        {
            //printf("stereo image; track feature on right image\n");
            vector<cv::Point2f> reverseLeftPts;
            vector<uchar> status, statusRightLeft;
            vector<float> err;
            // cur left ---- cur right
            cv::calcOpticalFlowPyrLK(cur_img, rightImg, cur_pts, cur_right_pts, status, err, cv::Size(21, 21), 3);
            // reverse check cur right ---- cur left
            if(FLOW_BACK)
            {
                cv::calcOpticalFlowPyrLK(rightImg, cur_img, cur_right_pts, reverseLeftPts, statusRightLeft, err, cv::Size(21, 21), 3);
                for(size_t i = 0; i < status.size(); i++)
                {
                    if(status[i] && statusRightLeft[i] && inBorder(cur_right_pts[i]) && distance(cur_pts[i], reverseLeftPts[i]) <= 0.5)
                        status[i] = 1;
                    else
                        status[i] = 0;
                }
            }

            ids_right = ids;
            reduceVector(cur_right_pts, status);
            reduceVector(ids_right, status);
            // only keep left-right pts
            /*
            reduceVector(cur_pts, status);
            reduceVector(ids, status);
            reduceVector(track_cnt, status);
            reduceVector(cur_un_pts, status);
            reduceVector(pts_velocity, status);
            */
            cur_un_right_pts = undistortedPts(cur_right_pts, m_camera[1]);
            right_pts_velocity = ptsVelocity(ids_right, cur_un_right_pts, cur_un_right_pts_map, prev_un_right_pts_map);
        }
        prev_un_right_pts_map = cur_un_right_pts_map;
    }
    if(SHOW_TRACK)
        drawTrack(cur_img, rightImg, ids, cur_pts, cur_right_pts, prevLeftPtsMap);

    prev_img = cur_img;
    prev_pts = cur_pts;
    prev_un_pts = cur_un_pts;
    prev_un_pts_map = cur_un_pts_map;
    prev_time = cur_time;
    hasPrediction = false;

    prevLeftPtsMap.clear();
    for(size_t i = 0; i < cur_pts.size(); i++)
        prevLeftPtsMap[ids[i]] = cur_pts[i];

    map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>> featureFrame;
    for (size_t i = 0; i < ids.size(); i++)
    {
        int feature_id = ids[i];
        double x, y ,z;
        x = cur_un_pts[i].x;
        y = cur_un_pts[i].y;
        z = 1;
        double p_u, p_v;
        p_u = cur_pts[i].x;
        p_v = cur_pts[i].y;
        int camera_id = 0;
        double velocity_x, velocity_y;
        velocity_x = pts_velocity[i].x;
        velocity_y = pts_velocity[i].y;

        Eigen::Matrix<double, 7, 1> xyz_uv_velocity;
        xyz_uv_velocity << x, y, z, p_u, p_v, velocity_x, velocity_y;
        featureFrame[feature_id].emplace_back(camera_id,  xyz_uv_velocity);
    }

    if (!_img1.empty() && stereo_cam)
    {
        for (size_t i = 0; i < ids_right.size(); i++)
        {
            int feature_id = ids_right[i];
            double x, y ,z;
            x = cur_un_right_pts[i].x;
            y = cur_un_right_pts[i].y;
            z = 1;
            double p_u, p_v;
            p_u = cur_right_pts[i].x;
            p_v = cur_right_pts[i].y;
            int camera_id = 1;
            double velocity_x, velocity_y;
            velocity_x = right_pts_velocity[i].x;
            velocity_y = right_pts_velocity[i].y;

            Eigen::Matrix<double, 7, 1> xyz_uv_velocity;
            xyz_uv_velocity << x, y, z, p_u, p_v, velocity_x, velocity_y;
            featureFrame[feature_id].emplace_back(camera_id,  xyz_uv_velocity);
        }
    }

    //printf("feature track whole time %f\n", t_r.toc());
    return featureFrame;
}

void FeatureTracker::rejectWithF()
{
    if (cur_pts.size() >= 8)
    {
        ROS_DEBUG("FM ransac begins");
        TicToc t_f;
        vector<cv::Point2f> un_cur_pts(cur_pts.size()), un_prev_pts(prev_pts.size());
        for (unsigned int i = 0; i < cur_pts.size(); i++)
        {
            Eigen::Vector3d tmp_p;
            m_camera[0]->liftProjective(Eigen::Vector2d(cur_pts[i].x, cur_pts[i].y), tmp_p);
            tmp_p.x() = FOCAL_LENGTH * tmp_p.x() / tmp_p.z() + col / 2.0;
            tmp_p.y() = FOCAL_LENGTH * tmp_p.y() / tmp_p.z() + row / 2.0;
            un_cur_pts[i] = cv::Point2f(tmp_p.x(), tmp_p.y());

            m_camera[0]->liftProjective(Eigen::Vector2d(prev_pts[i].x, prev_pts[i].y), tmp_p);
            tmp_p.x() = FOCAL_LENGTH * tmp_p.x() / tmp_p.z() + col / 2.0;
            tmp_p.y() = FOCAL_LENGTH * tmp_p.y() / tmp_p.z() + row / 2.0;
            un_prev_pts[i] = cv::Point2f(tmp_p.x(), tmp_p.y());
        }

        vector<uchar> status;
        cv::findFundamentalMat(un_cur_pts, un_prev_pts, cv::FM_RANSAC, F_THRESHOLD, 0.99, status);
        int size_a = cur_pts.size();
        reduceVector(prev_pts, status);
        reduceVector(cur_pts, status);
        reduceVector(ids, status);
        reduceVector(track_cnt, status);
        ROS_DEBUG("FM ransac: %d -> %lu: %f", size_a, cur_pts.size(), 1.0 * cur_pts.size() / size_a);
        ROS_DEBUG("FM ransac costs: %fms", t_f.toc());
    }
}

void FeatureTracker::readIntrinsicParameter(const vector<string> &calib_file)
{
    for (size_t i = 0; i < calib_file.size(); i++)
    {
        ROS_INFO("reading paramerter of camera %s", calib_file[i].c_str());
        camodocal::CameraPtr camera = CameraFactory::instance()->generateCameraFromYamlFile(calib_file[i]);
        m_camera.push_back(camera);
    }
    if (calib_file.size() == 2)
        stereo_cam = 1;
}

void FeatureTracker::showUndistortion(const string &name)
{
    cv::Mat undistortedImg(row + 600, col + 600, CV_8UC1, cv::Scalar(0));
    vector<Eigen::Vector2d> distortedp, undistortedp;
    for (int i = 0; i < col; i++)
        for (int j = 0; j < row; j++)
        {
            Eigen::Vector2d a(i, j);
            Eigen::Vector3d b;
            m_camera[0]->liftProjective(a, b);
            distortedp.push_back(a);
            undistortedp.push_back(Eigen::Vector2d(b.x() / b.z(), b.y() / b.z()));
            //printf("%f,%f->%f,%f,%f\n)\n", a.x(), a.y(), b.x(), b.y(), b.z());
        }
    for (int i = 0; i < int(undistortedp.size()); i++)
    {
        cv::Mat pp(3, 1, CV_32FC1);
        pp.at<float>(0, 0) = undistortedp[i].x() * FOCAL_LENGTH + col / 2;
        pp.at<float>(1, 0) = undistortedp[i].y() * FOCAL_LENGTH + row / 2;
        pp.at<float>(2, 0) = 1.0;
        //cout << trackerData[0].K << endl;
        //printf("%lf %lf\n", p.at<float>(1, 0), p.at<float>(0, 0));
        //printf("%lf %lf\n", pp.at<float>(1, 0), pp.at<float>(0, 0));
        if (pp.at<float>(1, 0) + 300 >= 0 && pp.at<float>(1, 0) + 300 < row + 600 && pp.at<float>(0, 0) + 300 >= 0 && pp.at<float>(0, 0) + 300 < col + 600)
        {
            undistortedImg.at<uchar>(pp.at<float>(1, 0) + 300, pp.at<float>(0, 0) + 300) = cur_img.at<uchar>(distortedp[i].y(), distortedp[i].x());
        }
        else
        {
            //ROS_ERROR("(%f %f) -> (%f %f)", distortedp[i].y, distortedp[i].x, pp.at<float>(1, 0), pp.at<float>(0, 0));
        }
    }
    // turn the following code on if you need
    // cv::imshow(name, undistortedImg);
    // cv::waitKey(0);
}

vector<cv::Point2f> FeatureTracker::undistortedPts(vector<cv::Point2f> &pts, camodocal::CameraPtr cam)
{
    vector<cv::Point2f> un_pts;
    for (unsigned int i = 0; i < pts.size(); i++)
    {
        Eigen::Vector2d a(pts[i].x, pts[i].y);
        Eigen::Vector3d b;
        cam->liftProjective(a, b);
        un_pts.push_back(cv::Point2f(b.x() / b.z(), b.y() / b.z()));
    }
    return un_pts;
}

vector<cv::Point2f> FeatureTracker::ptsVelocity(vector<int> &ids, vector<cv::Point2f> &pts, 
                                            map<int, cv::Point2f> &cur_id_pts, map<int, cv::Point2f> &prev_id_pts)
{
    vector<cv::Point2f> pts_velocity;
    cur_id_pts.clear();
    for (unsigned int i = 0; i < ids.size(); i++)
    {
        cur_id_pts.insert(make_pair(ids[i], pts[i]));
    }

    // caculate points velocity
    if (!prev_id_pts.empty())
    {
        double dt = cur_time - prev_time;
        
        for (unsigned int i = 0; i < pts.size(); i++)
        {
            std::map<int, cv::Point2f>::iterator it;
            it = prev_id_pts.find(ids[i]);
            if (it != prev_id_pts.end())
            {
                double v_x = (pts[i].x - it->second.x) / dt;
                double v_y = (pts[i].y - it->second.y) / dt;
                pts_velocity.push_back(cv::Point2f(v_x, v_y));
            }
            else
                pts_velocity.push_back(cv::Point2f(0, 0));

        }
    }
    else
    {
        for (unsigned int i = 0; i < cur_pts.size(); i++)
        {
            pts_velocity.push_back(cv::Point2f(0, 0));
        }
    }
    return pts_velocity;
}

void FeatureTracker::drawTrack(const cv::Mat &imLeft, const cv::Mat &imRight, 
                               vector<int> &curLeftIds,
                               vector<cv::Point2f> &curLeftPts, 
                               vector<cv::Point2f> &curRightPts,
                               map<int, cv::Point2f> &prevLeftPtsMap)
{
    //int rows = imLeft.rows;
    int cols = imLeft.cols;
    if (!imRight.empty() && stereo_cam)
        cv::hconcat(imLeft, imRight, imTrack);
    else
        imTrack = imLeft.clone();
    cv::cvtColor(imTrack, imTrack, cv::COLOR_GRAY2RGB);

    for (size_t j = 0; j < curLeftPts.size(); j++)
    {
        const bool is_new_point = j < track_cnt.size() && track_cnt[j] <= 1;
        const cv::Scalar color = is_new_point ? cv::Scalar(0, 255, 255) : cv::Scalar(0, 165, 255);
        cv::circle(imTrack, curLeftPts[j], 2, color, 2);
    }

    std::ostringstream overlay;
    overlay << "total=" << last_total_feature_count_
            << " tracked=" << last_tracked_feature_count_
            << " new=" << last_new_feature_count_
            << " cover=" << std::fixed << std::setprecision(2) << last_grid_coverage_ratio_;
    cv::rectangle(imTrack, cv::Point(5, 5), cv::Point(365, 34), cv::Scalar(0, 0, 0), -1);
    cv::putText(imTrack, overlay.str(), cv::Point(12, 26), cv::FONT_HERSHEY_SIMPLEX, 0.55,
                cv::Scalar(255, 255, 255), 1, cv::LINE_AA);
    cv::putText(imTrack, "tracked=orange new=yellow", cv::Point(12, 52), cv::FONT_HERSHEY_SIMPLEX, 0.45,
                cv::Scalar(255, 255, 255), 1, cv::LINE_AA);

    // Read-only low-texture warning. We do NOT fabricate points to fill the
    // frame here; we only flag that the scene is texture-poor so the run log /
    // debug image make it visible when the estimator should be leaning on the
    // IMU. Whether to act on this downstream is decided after reviewing bags.
    if (FRONTEND_LOW_TEX_COVERAGE > 0.0 &&
        last_grid_coverage_ratio_ < FRONTEND_LOW_TEX_COVERAGE)
    {
        cv::rectangle(imTrack, cv::Point(5, 60), cv::Point(365, 88), cv::Scalar(0, 0, 128), -1);
        cv::putText(imTrack, "LOW-TEX: trust IMU", cv::Point(12, 82), cv::FONT_HERSHEY_SIMPLEX, 0.6,
                    cv::Scalar(0, 0, 255), 2, cv::LINE_AA);
        ROS_WARN_STREAM_THROTTLE(1.0, "FRONTEND_LOW_TEX coverage=" << std::fixed
                                 << std::setprecision(3) << last_grid_coverage_ratio_
                                 << " < " << FRONTEND_LOW_TEX_COVERAGE
                                 << " total=" << last_total_feature_count_);
    }
    if (!imRight.empty() && stereo_cam)
    {
        for (size_t i = 0; i < curRightPts.size(); i++)
        {
            cv::Point2f rightPt = curRightPts[i];
            rightPt.x += cols;
            cv::circle(imTrack, rightPt, 2, cv::Scalar(0, 255, 0), 2);
            //cv::Point2f leftPt = curLeftPtsTrackRight[i];
            //cv::line(imTrack, leftPt, rightPt, cv::Scalar(0, 255, 0), 1, 8, 0);
        }
    }
    
    map<int, cv::Point2f>::iterator mapIt;
    for (size_t i = 0; i < curLeftIds.size(); i++)
    {
        int id = curLeftIds[i];
        mapIt = prevLeftPtsMap.find(id);
        if(mapIt != prevLeftPtsMap.end())
        {
            cv::arrowedLine(imTrack, curLeftPts[i], mapIt->second, cv::Scalar(0, 255, 0), 1, 8, 0, 0.2);
        }
    }

    //draw prediction
    /*
    for(size_t i = 0; i < predict_pts_debug.size(); i++)
    {
        cv::circle(imTrack, predict_pts_debug[i], 2, cv::Scalar(0, 170, 255), 2);
    }
    */
    //printf("predict pts size %d \n", (int)predict_pts_debug.size());

    //cv::Mat imCur2Compress;
    //cv::resize(imCur2, imCur2Compress, cv::Size(cols, rows / 2));
}


void FeatureTracker::setPrediction(map<int, Eigen::Vector3d> &predictPts)
{
    hasPrediction = true;
    predict_pts.clear();
    predict_pts_debug.clear();
    map<int, Eigen::Vector3d>::iterator itPredict;
    for (size_t i = 0; i < ids.size(); i++)
    {
        //printf("prevLeftId size %d prevLeftPts size %d\n",(int)prevLeftIds.size(), (int)prevLeftPts.size());
        int id = ids[i];
        itPredict = predictPts.find(id);
        if (itPredict != predictPts.end())
        {
            Eigen::Vector2d tmp_uv;
            m_camera[0]->spaceToPlane(itPredict->second, tmp_uv);
            predict_pts.push_back(cv::Point2f(tmp_uv.x(), tmp_uv.y()));
            predict_pts_debug.push_back(cv::Point2f(tmp_uv.x(), tmp_uv.y()));
        }
        else
            predict_pts.push_back(prev_pts[i]);
    }
}


void FeatureTracker::removeOutliers(set<int> &removePtsIds)
{
    std::set<int>::iterator itSet;
    vector<uchar> status;
    for (size_t i = 0; i < ids.size(); i++)
    {
        itSet = removePtsIds.find(ids[i]);
        if(itSet != removePtsIds.end())
            status.push_back(0);
        else
            status.push_back(1);
    }

    reduceVector(prev_pts, status);
    reduceVector(ids, status);
    reduceVector(track_cnt, status);
}


cv::Mat FeatureTracker::getTrackImage()
{
    return imTrack;
}
