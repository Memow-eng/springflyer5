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

extern int GOOD_FEATURE_ENABLE;
extern int GOOD_FEATURE_BUDGET;
extern double GOOD_FEATURE_MIN_SCALE;
extern int GOOD_FEATURE_MIN_TRACK_LENGTH;

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
