/*******************************************************
 * Copyright (C) 2019, Aerial Robotics Group, Hong Kong University of Science and Technology
 * 
 * This file is part of VINS.
 * 
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *******************************************************/

#pragma once

#include <ros/ros.h>
#include <vector>
#include <eigen3/Eigen/Dense>
#include "../utility/utility.h"
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>
#include <fstream>
#include <map>

using namespace std;

const double FOCAL_LENGTH = 460.0;
const int WINDOW_SIZE = 10;
const int NUM_OF_F = 1000;
typedef Eigen::Matrix<double, 8, 1> FeatureObservation;
//#define UNIT_SPHERE_ERROR

extern double INIT_DEPTH;
extern double MIN_PARALLAX;
extern int ESTIMATE_EXTRINSIC;

extern double ACC_N, ACC_W;
extern double GYR_N, GYR_W;

extern std::vector<Eigen::Matrix3d> RIC;
extern std::vector<Eigen::Vector3d> TIC;
extern Eigen::Vector3d G;

extern double BIAS_ACC_THRESHOLD;
extern double BIAS_GYR_THRESHOLD;
extern double SOLVER_TIME;
extern int NUM_ITERATIONS;
extern int MARGINALIZATION_NUM_THREADS;
extern std::string EX_CALIB_RESULT_PATH;
extern std::string VINS_RESULT_PATH;
extern std::string OUTPUT_FOLDER;
extern std::string IMU_TOPIC;
extern double TD;
extern int ESTIMATE_TD;
extern int ROLLING_SHUTTER;
extern int ROW, COL;
extern int NUM_OF_CAM;
extern int STEREO;
extern int USE_IMU;
extern int MULTIPLE_THREAD;
extern int OPENCV_NUM_THREADS;
extern int IMAGE_SYNC_SLEEP_MS;
// pts_gt for debug purpose;
extern map<int, Eigen::Vector3d> pts_gt;

extern std::string IMAGE0_TOPIC, IMAGE1_TOPIC;
extern std::string FISHEYE_MASK;
extern std::vector<std::string> CAM_NAMES;
extern int MAX_CNT;
extern int MIN_DIST;
extern double F_THRESHOLD;
extern int SHOW_TRACK;
extern int FLOW_BACK;
extern int FEATURE_LOG_ENABLE;
extern std::string FEATURE_LOG_PATH;
extern int FEATURE_LOG_PRINT_EVERY;
extern int FEATURE_LOG_FLUSH_EVERY;

extern int FRONTEND_ADAPTIVE_FEATURE;
extern int FRONTEND_GRADIENT_POINTS;
extern double FRONTEND_LOW_QUALITY;
extern double FRONTEND_MIN_QUALITY;
extern double FRONTEND_DEGRADED_MIN_DIST_RATIO;
extern int FRONTEND_GRADIENT_GRID;
extern double FRONTEND_GRADIENT_MIN;
extern double FRONTEND_FB_THRESHOLD;
extern double FRONTEND_LK_MAX_ERROR;
extern double FRONTEND_TRACK_MIN_EIGEN;
extern int FRONTEND_RANSAC;
extern int FRONTEND_RANSAC_STRICT;
extern int FRONTEND_RANSAC_MIN_POINTS;
extern int FRONTEND_RANSAC_MIN_INLIERS;
extern double FRONTEND_RANSAC_MIN_RATIO;
extern int FRONTEND_USE_ROT_RANSAC;
extern double FRONTEND_LOWPAR_PARALLAX;
extern double FRONTEND_LOWPAR_RESIDUAL;
extern double FRONTEND_ROT_RANSAC_THRESH;
extern int FRONTEND_ROT_RANSAC_ITERS;
extern double FRONTEND_LOWPAR_COND_RATIO;
extern int FRONTEND_QUALITY_GRID_COLS;
extern int FRONTEND_QUALITY_GRID_ROWS;
extern int FRONTEND_QUALITY_MIN_TRACKED;
extern int FRONTEND_MIN_QUALITY_POINTS;
extern double FRONTEND_MIN_LK_KEEP_RATIO;
extern double FRONTEND_MIN_COVERAGE_RATIO;
extern double FRONTEND_QUALITY_MIN_EIGEN;
extern int FRONTEND_CELL_GRID_ROWS;
extern int FRONTEND_CELL_GRID_COLS;
extern int FRONTEND_CELL_LOW_TEX_PASS;
extern double FRONTEND_CELL_LOW_QUALITY_SCALE;
extern double STEREO_MAX_VERTICAL_DIFF;
extern double STEREO_MIN_DISPARITY;
extern double STEREO_MAX_DISPARITY;

extern int GOOD_FEATURE_ENABLE;
extern int GOOD_FEATURE_BUDGET;
extern double GOOD_FEATURE_MIN_SCALE;
extern int GOOD_FEATURE_MIN_TRACK_LENGTH;
extern double GOOD_FEATURE_MIN_QUALITY;
extern double GOOD_FEATURE_NEW_MIN_QUALITY;
extern int GOOD_FEATURE_NEW_MIN_TRACK_LENGTH;
extern double GOOD_FEATURE_MIN_PARALLAX;
extern double GOOD_FEATURE_MAX_REPROJ_ERROR;

extern double QUALITY_DEGRADED_MEDIAN;
extern double QUALITY_DEGRADED_BAD_RATIO;
extern double QUALITY_DEGRADED_HQ_LONG_RATIO;
extern double QUALITY_DEGRADED_NEW_RATIO;
extern int QUALITY_DEGRADED_DEBOUNCE;

extern int BLIND_ENABLE;
extern int BLIND_ENTER_TRACK_NUM;
extern int BLIND_EXIT_TRACK_NUM;
extern int BLIND_DEGRADED_TRACK_NUM;
extern double BLIND_PARALLAX_THRESHOLD;
extern double BLIND_BIAS_ACC_SIGMA;
extern double BLIND_BIAS_GYR_SIGMA;
extern double BLIND_BIAS_RELAX_RATE;
extern double BLIND_BIAS_RELAX_MAX;
extern double BLIND_BIAS_ACC_MAX;
extern double BLIND_BIAS_GYR_MAX;
extern int NOMINAL_BIAS_PRIOR_ENABLE;
extern double NOMINAL_ACC_BIAS_SIGMA;
extern double BLIND_VELOCITY_PRIOR_WEIGHT;
extern double BLIND_TILT_WEIGHT;
extern double BLIND_TILT_MAX_ACC_DEV;
extern double BLIND_TILT_MAX_GYR;
extern double BLIND_ZUPT_WEIGHT;
extern double BLIND_ZUPT_MAX_ACC_VAR;
extern double BLIND_ZUPT_MAX_GYR;
extern double BLIND_THRUST_WEIGHT;
extern double BLIND_THRUST_HOVER_ACC;
extern double BLIND_THRUST_HOVER_THROTTLE;
extern double BLIND_THRUST_MIN_DT;
extern double BLIND_VISUAL_WEIGHT_DEGRADED;
extern int LOW_FLOW_ZUPT_ENABLE;
extern double LOW_FLOW_ZUPT_FLOW;
extern int LOW_FLOW_ZUPT_MIN_TRACKS;
extern double LOW_FLOW_ZUPT_WEIGHT;
extern int LOW_FLOW_ZUPT_WINDOW;
extern int HOME_LOOP_ENABLE;
extern double HOME_LOOP_MIN_TRAVEL;
extern double HOME_LOOP_CAPTURE_RADIUS;
extern double HOME_LOOP_GAIN;
extern int HOME_LOOP_MIN_STATIC_FRAMES;
extern int BLIND_PTS_OK;
extern int BLIND_PTS_MIN;
extern int BLIND_PTS_BLIND;
extern double BLIND_FLOW_DYN;
extern double BLIND_FLOW_STATIC;
extern int FAILURE_GRACE_FRAMES;
extern double FAILURE_POSE_JUMP_HARD;
extern int BLIND_DEBOUNCE_FRAMES;

void readParameters(std::string config_file);

enum SIZE_PARAMETERIZATION
{
    SIZE_POSE = 7,
    SIZE_SPEEDBIAS = 9,
    SIZE_FEATURE = 1
};

enum StateOrder
{
    O_P = 0,
    O_R = 3,
    O_V = 6,
    O_BA = 9,
    O_BG = 12
};

enum NoiseOrder
{
    O_AN = 0,
    O_GN = 3,
    O_AW = 6,
    O_GW = 9
};
