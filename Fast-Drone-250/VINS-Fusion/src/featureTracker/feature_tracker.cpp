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
#include <fstream>   // ===== FEATURE LOGGING (added) =====
#include <cstring>   // ===== FEATURE LOGGING (added) =====

// ===================== FEATURE LOGGING (added for distribution analysis) =====================
// 输出每帧"最终送进后端的"左相机特征点 (经过 光流 + 前后向 + setMask + 补点 之后).
//   1) CSV : 每点一行  t,id,u,v,track_cnt   -> 事后用 Python 画 数量/空间分布/寿命
//   2) 终端: 每 N 帧打印一次网格热力, 实时粗看空间分布是否病态
// 不改动任何原有跟踪逻辑, 纯旁路记录.
namespace
{
const char *FEAT_LOG_CSV     = "/tmp/vins_feature_points.csv"; // 输出路径, 按需改
const int   FEAT_GRID_R      = 6;   // 网格行数
const int   FEAT_GRID_C      = 8;   // 网格列数
const int   FEAT_PRINT_EVERY = 10;  // 每多少帧在终端打印一次网格 (CSV 每帧都写)

void logFeatureStats(double t,
                     const std::vector<cv::Point2f> &pts,
                     const std::vector<int> &ids,
                     const std::vector<int> &track_cnt,
                     int row, int col)
{
    // ---- CSV: 每个点一行 ----
    static std::ofstream fout;
    static bool inited = false;
    if (!inited)
    {
        fout.open(FEAT_LOG_CSV, std::ios::out | std::ios::trunc);
        if (fout.is_open())
            fout << "t,id,u,v,track_cnt\n";
        else
            printf("[FT][WARN] cannot open %s for logging\n", FEAT_LOG_CSV);
        inited = true;
    }
    if (fout.is_open())
    {
        fout.setf(std::ios::fixed);
        for (size_t i = 0; i < pts.size() && i < ids.size() && i < track_cnt.size(); i++)
            fout << t << "," << ids[i] << ","
                 << pts[i].x << "," << pts[i].y << ","
                 << track_cnt[i] << "\n";
        fout.flush();
    }

    // ---- 终端网格热力 ----
    if (row <= 0 || col <= 0)
        return;

    int grid[FEAT_GRID_R][FEAT_GRID_C];
    memset(grid, 0, sizeof(grid));
    int cell_h = std::max(1, row / FEAT_GRID_R);
    int cell_w = std::max(1, col / FEAT_GRID_C);
    for (const auto &p : pts)
    {
        int gr = std::min((int)(p.y / cell_h), FEAT_GRID_R - 1);
        int gc = std::min((int)(p.x / cell_w), FEAT_GRID_C - 1);
        if (gr < 0 || gc < 0)
            continue;
        grid[gr][gc]++;
    }
    int empty_cells = 0, max_cell = 0;
    for (int r = 0; r < FEAT_GRID_R; r++)
        for (int c = 0; c < FEAT_GRID_C; c++)
        {
            if (grid[r][c] == 0)
                empty_cells++;
            max_cell = std::max(max_cell, grid[r][c]);
        }

    static int frame_cnt = 0;
    frame_cnt++;
    if (frame_cnt % FEAT_PRINT_EVERY == 0)
    {
        // total / 空格子数(越大分布越差) / 最满格子点数(越大越扎堆)
        printf("[FT] total=%zu  empty_cells=%d/%d  max_cell=%d  grid:\n",
               pts.size(), empty_cells, FEAT_GRID_R * FEAT_GRID_C, max_cell);
        for (int r = 0; r < FEAT_GRID_R; r++)
        {
            printf("[FT]   ");
            for (int c = 0; c < FEAT_GRID_C; c++)
                printf("%3d ", grid[r][c]);
            printf("\n");
        }
    }
}
} // anonymous namespace
// ===================== end FEATURE LOGGING =====================

FrontendQuality::FrontendQuality()
    : prev_points(0),
      tracked_after_lk(0),
      tracked_after_ransac(0),
      new_points(0),
      total_points(0),
      occupied_cells(0),
      grid_cols(0),
      grid_rows(0),
      lk_keep_ratio(1.0),
      mean_lk_error(-1.0),
      mean_fb_error(-1.0),
      mean_track_eigen(-1.0),
      coverage_ratio(0.0),
      low_tracking_quality(false),
      weak_texture(false),
      poor_distribution(false),
      ransac_rejected(false)
{
}

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
    last_quality = FrontendQuality();
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

void FeatureTracker::addAdaptiveCorners(int need_cnt)
{
    if (need_cnt <= 0)
        return;

    const double qualities[3] = {0.01, FRONTEND_LOW_QUALITY, FRONTEND_MIN_QUALITY};
    const int base_min_dist = std::max(5, MIN_DIST);
    const int low_min_dist = std::max(5, static_cast<int>(MIN_DIST * FRONTEND_DEGRADED_MIN_DIST_RATIO));

    for (int pass = 0; pass < 3 && static_cast<int>(n_pts.size()) < need_cnt; pass++)
    {
        const int min_dist = pass == 0 ? base_min_dist : low_min_dist;
        vector<cv::Point2f> tmp_pts;
        int remain = need_cnt - static_cast<int>(n_pts.size());
        cv::goodFeaturesToTrack(cur_img, tmp_pts, remain, qualities[pass], min_dist, mask);

        for (auto &p : tmp_pts)
        {
            if (!inBorder(p))
                continue;
            if (mask.at<uchar>(p) == 0)
                continue;
            n_pts.push_back(p);
            cv::circle(mask, p, min_dist, 0, -1);
            if (static_cast<int>(n_pts.size()) >= need_cnt)
                break;
        }
    }
}

void FeatureTracker::addGradientFeatures(int need_cnt)
{
    if (need_cnt <= 0 || cur_img.empty() || mask.empty())
        return;

    cv::Mat eig;
    cv::cornerMinEigenVal(cur_img, eig, 3, 3);

    double global_max = 0.0;
    cv::minMaxLoc(eig, nullptr, &global_max, nullptr, nullptr, mask);

    const double thresh = std::max(static_cast<double>(FRONTEND_GRADIENT_MIN),
                                   FRONTEND_MIN_QUALITY * global_max);

    struct Candidate
    {
        cv::Point2f pt;
        float score;
    };
    vector<Candidate> candidates;

    const int grid = std::max(8, FRONTEND_GRADIENT_GRID);
    for (int y = 0; y < row; y += grid)
    {
        for (int x = 0; x < col; x += grid)
        {
            cv::Rect roi(x, y, std::min(grid, col - x), std::min(grid, row - y));
            if (roi.width <= 0 || roi.height <= 0)
                continue;
            double mv = 0.0;
            cv::Point ml;
            cv::minMaxLoc(eig(roi), nullptr, &mv, nullptr, &ml, mask(roi));
            if (mv < thresh)
                continue;
            cv::Point2f p(static_cast<float>(x + ml.x), static_cast<float>(y + ml.y));
            if (inBorder(p))
                candidates.push_back({p, static_cast<float>(mv)});
        }
    }

    std::sort(candidates.begin(), candidates.end(),
              [](const Candidate &a, const Candidate &b)
              {
                  return a.score > b.score;
              });

    const int md = std::max(5, static_cast<int>(MIN_DIST * FRONTEND_DEGRADED_MIN_DIST_RATIO));
    for (auto &c : candidates)
    {
        if (static_cast<int>(n_pts.size()) >= need_cnt)
            break;
        if (mask.at<uchar>(c.pt) == 0)
            continue;
        n_pts.push_back(c.pt);
        cv::circle(mask, c.pt, md, 0, -1);
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
    last_quality = FrontendQuality();
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
        last_quality.prev_points = static_cast<int>(prev_pts.size());
        if(hasPrediction)
        {
            cur_pts = predict_pts;
            cv::calcOpticalFlowPyrLK(prev_img, cur_img, prev_pts, cur_pts, status, err, cv::Size(21, 21), 3, 
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
                if(status[i] && reverse_status[i] && distance(prev_pts[i], reverse_pts[i]) <= 0.5)
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
        int kept_cnt = 0;
        for (uchar s : status)
            kept_cnt += s ? 1 : 0;
        last_quality.tracked_after_lk = kept_cnt;
        last_quality.lk_keep_ratio = last_quality.prev_points > 0
                                         ? static_cast<double>(kept_cnt) / static_cast<double>(last_quality.prev_points)
                                         : 1.0;
        last_quality.low_tracking_quality =
            last_quality.prev_points >= FRONTEND_QUALITY_MIN_TRACKED &&
            last_quality.lk_keep_ratio < FRONTEND_MIN_LK_KEEP_RATIO;
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
        //rejectWithF();
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
            if (FRONTEND_ADAPTIVE_FEATURE)
                addAdaptiveCorners(n_max_cnt);
            else
                cv::goodFeaturesToTrack(cur_img, n_pts, n_max_cnt, 0.01, MIN_DIST, mask);

            if (FRONTEND_GRADIENT_POINTS && static_cast<int>(cur_pts.size() + n_pts.size()) < MAX_CNT)
                addGradientFeatures(MAX_CNT - static_cast<int>(cur_pts.size()));
        }
        else
            n_pts.clear();
        ROS_DEBUG("detect feature costs: %f ms", t_t.toc());

        for (auto &p : n_pts)
        {
            cur_pts.push_back(p);
            ids.push_back(n_id++);
            track_cnt.push_back(1);
        }
        last_quality.new_points = static_cast<int>(n_pts.size());
        last_quality.tracked_after_ransac = static_cast<int>(cur_pts.size());
        last_quality.total_points = static_cast<int>(cur_pts.size());
        if (!cur_pts.empty() && !cur_img.empty())
        {
            cv::Mat eig;
            cv::cornerMinEigenVal(cur_img, eig, 3, 3);
            double eig_sum = 0.0;
            int eig_num = 0;
            for (const auto &p : cur_pts)
            {
                int x = cvRound(p.x);
                int y = cvRound(p.y);
                if (0 <= x && x < eig.cols && 0 <= y && y < eig.rows)
                {
                    eig_sum += eig.at<float>(y, x);
                    eig_num++;
                }
            }
            if (eig_num > 0)
                last_quality.mean_track_eigen = eig_sum / static_cast<double>(eig_num);
        }
        last_quality.grid_rows = std::max(1, FRONTEND_QUALITY_GRID_ROWS);
        last_quality.grid_cols = std::max(1, FRONTEND_QUALITY_GRID_COLS);
        if (row > 0 && col > 0)
        {
            vector<uchar> occupied(last_quality.grid_rows * last_quality.grid_cols, 0);
            for (const auto &p : cur_pts)
            {
                int gx = std::min(last_quality.grid_cols - 1,
                                  std::max(0, static_cast<int>(p.x * last_quality.grid_cols / col)));
                int gy = std::min(last_quality.grid_rows - 1,
                                  std::max(0, static_cast<int>(p.y * last_quality.grid_rows / row)));
                occupied[gy * last_quality.grid_cols + gx] = 1;
            }
            for (uchar s : occupied)
                last_quality.occupied_cells += s ? 1 : 0;
            last_quality.coverage_ratio =
                static_cast<double>(last_quality.occupied_cells) /
                static_cast<double>(last_quality.grid_rows * last_quality.grid_cols);
        }
        last_quality.weak_texture =
            last_quality.total_points < FRONTEND_MIN_QUALITY_POINTS ||
            (last_quality.mean_track_eigen >= 0.0 &&
             last_quality.mean_track_eigen < FRONTEND_QUALITY_MIN_EIGEN);
        last_quality.poor_distribution =
            last_quality.total_points >= FRONTEND_MIN_QUALITY_POINTS &&
            last_quality.coverage_ratio < FRONTEND_MIN_COVERAGE_RATIO;
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

    // ===== FEATURE LOGGING (added) =====
    // 此处 cur_pts / ids / track_cnt 已是最终送进后端的左相机点 (一一对齐).
    logFeatureStats(cur_time, cur_pts, ids, track_cnt, row, col);
    // ===================================

    //printf("feature track whole time %f\n", t_r.toc());
    return featureFrame;
}

const FrontendQuality &FeatureTracker::getLastFrontendQuality() const
{
    return last_quality;
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
        reduceVector(cur_un_pts, status);
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
        double len = std::min(1.0, 1.0 * track_cnt[j] / 20);
        cv::circle(imTrack, curLeftPts[j], 2, cv::Scalar(255 * (1 - len), 0, 255 * len), 2);
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

    std::vector<cv::Point2f> tmp(prev_pts.size());
    std::vector<char> has(prev_pts.size(), 0);
    std::vector<float> dxs, dys;

    for (size_t i = 0; i < ids.size() && i < prev_pts.size(); i++)
    {
        auto it = predictPts.find(ids[i]);
        if (it != predictPts.end())
        {
            Eigen::Vector2d uv;
            m_camera[0]->spaceToPlane(it->second, uv);
            tmp[i] = cv::Point2f(static_cast<float>(uv.x()), static_cast<float>(uv.y()));
            has[i] = 1;
            dxs.push_back(tmp[i].x - prev_pts[i].x);
            dys.push_back(tmp[i].y - prev_pts[i].y);
        }
    }

    float mdx = 0.f, mdy = 0.f;
    if (!dxs.empty())
    {
        size_t m = dxs.size() / 2;
        std::nth_element(dxs.begin(), dxs.begin() + m, dxs.end());
        mdx = dxs[m];
        std::nth_element(dys.begin(), dys.begin() + m, dys.end());
        mdy = dys[m];
    }

    for (size_t i = 0; i < prev_pts.size(); i++)
    {
        cv::Point2f p = has[i] ? tmp[i]
                               : cv::Point2f(prev_pts[i].x + mdx, prev_pts[i].y + mdy);
        predict_pts.push_back(p);
        predict_pts_debug.push_back(p);
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
