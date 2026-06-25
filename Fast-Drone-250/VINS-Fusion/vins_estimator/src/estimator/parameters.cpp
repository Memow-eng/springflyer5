/*******************************************************
 * Copyright (C) 2019, Aerial Robotics Group, Hong Kong University of Science and Technology
 * 
 * This file is part of VINS.
 * 
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *******************************************************/

#include "parameters.h"
#include <cerrno>
#include <sys/stat.h>
#include <sys/types.h>

double INIT_DEPTH;
double MIN_PARALLAX;
double ACC_N, ACC_W;
double GYR_N, GYR_W;

std::vector<Eigen::Matrix3d> RIC;
std::vector<Eigen::Vector3d> TIC;

Eigen::Vector3d G{0.0, 0.0, 9.8};

double BIAS_ACC_THRESHOLD;
double BIAS_GYR_THRESHOLD;
double SOLVER_TIME;
int NUM_ITERATIONS;
int MARGINALIZATION_NUM_THREADS = 2;
int ESTIMATE_EXTRINSIC;
int ESTIMATE_TD;
int ROLLING_SHUTTER;
std::string EX_CALIB_RESULT_PATH;
std::string VINS_RESULT_PATH;
std::string OUTPUT_FOLDER;
std::string IMU_TOPIC;
int ROW, COL;
double TD;
int NUM_OF_CAM;
int STEREO;
int USE_IMU;
int MULTIPLE_THREAD;
int OPENCV_NUM_THREADS = 2;
int IMAGE_SYNC_SLEEP_MS = 3;
map<int, Eigen::Vector3d> pts_gt;
std::string IMAGE0_TOPIC, IMAGE1_TOPIC;
std::string FISHEYE_MASK;
std::vector<std::string> CAM_NAMES;
int MAX_CNT;
int MIN_DIST;
double F_THRESHOLD;
int SHOW_TRACK;
int FLOW_BACK;
int FEATURE_LOG_ENABLE = 0;
std::string FEATURE_LOG_PATH = "/tmp/vins_feature_points.csv";
int FEATURE_LOG_PRINT_EVERY = 10;
int FEATURE_LOG_FLUSH_EVERY = 1;

int FRONTEND_ADAPTIVE_FEATURE = 1;
int FRONTEND_GRADIENT_POINTS = 1;
double FRONTEND_LOW_QUALITY = 0.006;
double FRONTEND_MIN_QUALITY = 0.002;
double FRONTEND_DEGRADED_MIN_DIST_RATIO = 0.6;
int FRONTEND_GRADIENT_GRID = 32;
double FRONTEND_GRADIENT_MIN = 1e-6;
double FRONTEND_FB_THRESHOLD = 0.5;
double FRONTEND_LK_MAX_ERROR = 45.0;
double FRONTEND_TRACK_MIN_EIGEN = 1e-6;
int FRONTEND_RANSAC = 1;
int FRONTEND_RANSAC_STRICT = 1;
int FRONTEND_RANSAC_MIN_POINTS = 12;
int FRONTEND_RANSAC_MIN_INLIERS = 8;
double FRONTEND_RANSAC_MIN_RATIO = 0.25;
int FRONTEND_QUALITY_GRID_COLS = 4;
int FRONTEND_QUALITY_GRID_ROWS = 3;
int FRONTEND_QUALITY_MIN_TRACKED = 20;
int FRONTEND_MIN_QUALITY_POINTS = 25;
double FRONTEND_MIN_LK_KEEP_RATIO = 0.35;
double FRONTEND_MIN_COVERAGE_RATIO = 0.25;
double FRONTEND_QUALITY_MIN_EIGEN = 1e-6;

int BLIND_ENABLE = 1;
int BLIND_ENTER_TRACK_NUM = 8;
int BLIND_EXIT_TRACK_NUM = 45;
int BLIND_DEGRADED_TRACK_NUM = 25;
double BLIND_PARALLAX_THRESHOLD = 1.0;
double BLIND_BIAS_ACC_SIGMA = 0.03;
double BLIND_BIAS_GYR_SIGMA = 0.002;
double BLIND_BIAS_RELAX_RATE = 0.25;
double BLIND_BIAS_RELAX_MAX = 3.0;
double BLIND_BIAS_ACC_MAX = 2.5;
double BLIND_BIAS_GYR_MAX = 1.0;
double BLIND_VELOCITY_PRIOR_WEIGHT = 0.5;
double BLIND_TILT_WEIGHT = 1.0;
double BLIND_TILT_MAX_ACC_DEV = 0.8;
double BLIND_TILT_MAX_GYR = 0.08;
double BLIND_ZUPT_WEIGHT = 2.0;
double BLIND_ZUPT_MAX_ACC_VAR = 0.05;
double BLIND_ZUPT_MAX_GYR = 0.03;
double BLIND_THRUST_WEIGHT = 0.0;
double BLIND_THRUST_HOVER_ACC = 9.805;
double BLIND_THRUST_HOVER_THROTTLE = 0.35;
double BLIND_THRUST_MIN_DT = 0.02;
double BLIND_VISUAL_WEIGHT_DEGRADED = 1.0;


template <typename T>
T readParam(ros::NodeHandle &n, std::string name)
{
    T ans;
    if (n.getParam(name, ans))
    {
        ROS_INFO_STREAM("Loaded " << name << ": " << ans);
    }
    else
    {
        ROS_ERROR_STREAM("Failed to load " << name);
        n.shutdown();
    }
    return ans;
}

template <typename T>
void readOptionalParam(cv::FileStorage &fsSettings, const std::string &name, T &value)
{
    cv::FileNode node = fsSettings[name];
    if (node.type() != cv::FileNode::NONE)
    {
        node >> value;
        ROS_INFO_STREAM("Loaded " << name << ": " << value);
    }
}

static bool createDirectoryRecursive(const std::string &path)
{
    if (path.empty())
        return false;

    std::string cur;
    size_t pos = 0;
    if (path[0] == '/')
    {
        cur = "/";
        pos = 1;
    }

    while (pos <= path.size())
    {
        size_t next = path.find('/', pos);
        std::string part = path.substr(pos, next == std::string::npos ? std::string::npos : next - pos);
        if (!part.empty())
        {
            if (cur.size() > 1 && cur.back() != '/')
                cur += "/";
            cur += part;
            if (mkdir(cur.c_str(), 0755) != 0 && errno != EEXIST)
                return false;
        }
        if (next == std::string::npos)
            break;
        pos = next + 1;
    }

    struct stat info;
    return stat(path.c_str(), &info) == 0 && S_ISDIR(info.st_mode);
}

void readParameters(std::string config_file)
{
    FILE *fh = fopen(config_file.c_str(),"r");
    if(fh == NULL){
        ROS_WARN("config_file dosen't exist; wrong config_file path");
        ROS_BREAK();
        return;          
    }
    fclose(fh);

    cv::FileStorage fsSettings(config_file, cv::FileStorage::READ);
    if(!fsSettings.isOpened())
    {
        std::cerr << "ERROR: Wrong path to settings" << std::endl;
    }

    fsSettings["image0_topic"] >> IMAGE0_TOPIC;
    fsSettings["image1_topic"] >> IMAGE1_TOPIC;
    MAX_CNT = fsSettings["max_cnt"];
    MIN_DIST = fsSettings["min_dist"];
    F_THRESHOLD = fsSettings["F_threshold"];
    SHOW_TRACK = fsSettings["show_track"];
    FLOW_BACK = fsSettings["flow_back"];
    readOptionalParam(fsSettings, "feature_log_enable", FEATURE_LOG_ENABLE);
    readOptionalParam(fsSettings, "feature_log_path", FEATURE_LOG_PATH);
    readOptionalParam(fsSettings, "feature_log_print_every", FEATURE_LOG_PRINT_EVERY);
    readOptionalParam(fsSettings, "feature_log_flush_every", FEATURE_LOG_FLUSH_EVERY);

    readOptionalParam(fsSettings, "blind_enable", BLIND_ENABLE);
    readOptionalParam(fsSettings, "blind_enter_track_num", BLIND_ENTER_TRACK_NUM);
    readOptionalParam(fsSettings, "blind_exit_track_num", BLIND_EXIT_TRACK_NUM);
    readOptionalParam(fsSettings, "blind_degraded_track_num", BLIND_DEGRADED_TRACK_NUM);
    readOptionalParam(fsSettings, "blind_parallax_threshold", BLIND_PARALLAX_THRESHOLD);
    readOptionalParam(fsSettings, "blind_bias_acc_sigma", BLIND_BIAS_ACC_SIGMA);
    readOptionalParam(fsSettings, "blind_bias_gyr_sigma", BLIND_BIAS_GYR_SIGMA);
    readOptionalParam(fsSettings, "blind_bias_relax_rate", BLIND_BIAS_RELAX_RATE);
    readOptionalParam(fsSettings, "blind_bias_relax_max", BLIND_BIAS_RELAX_MAX);
    readOptionalParam(fsSettings, "blind_bias_acc_max", BLIND_BIAS_ACC_MAX);
    readOptionalParam(fsSettings, "blind_bias_gyr_max", BLIND_BIAS_GYR_MAX);
    readOptionalParam(fsSettings, "blind_velocity_prior_weight", BLIND_VELOCITY_PRIOR_WEIGHT);
    readOptionalParam(fsSettings, "blind_tilt_weight", BLIND_TILT_WEIGHT);
    readOptionalParam(fsSettings, "blind_tilt_max_acc_dev", BLIND_TILT_MAX_ACC_DEV);
    readOptionalParam(fsSettings, "blind_tilt_max_gyr", BLIND_TILT_MAX_GYR);
    readOptionalParam(fsSettings, "blind_zupt_weight", BLIND_ZUPT_WEIGHT);
    readOptionalParam(fsSettings, "blind_zupt_max_acc_var", BLIND_ZUPT_MAX_ACC_VAR);
    readOptionalParam(fsSettings, "blind_zupt_max_gyr", BLIND_ZUPT_MAX_GYR);
    readOptionalParam(fsSettings, "blind_thrust_weight", BLIND_THRUST_WEIGHT);
    readOptionalParam(fsSettings, "blind_thrust_hover_acc", BLIND_THRUST_HOVER_ACC);
    readOptionalParam(fsSettings, "blind_thrust_hover_throttle", BLIND_THRUST_HOVER_THROTTLE);
    readOptionalParam(fsSettings, "blind_thrust_min_dt", BLIND_THRUST_MIN_DT);
    readOptionalParam(fsSettings, "blind_visual_weight_degraded", BLIND_VISUAL_WEIGHT_DEGRADED);
    readOptionalParam(fsSettings, "frontend_adaptive_feature", FRONTEND_ADAPTIVE_FEATURE);
    readOptionalParam(fsSettings, "frontend_gradient_points", FRONTEND_GRADIENT_POINTS);
    readOptionalParam(fsSettings, "frontend_low_quality", FRONTEND_LOW_QUALITY);
    readOptionalParam(fsSettings, "frontend_min_quality", FRONTEND_MIN_QUALITY);
    readOptionalParam(fsSettings, "frontend_degraded_min_dist_ratio", FRONTEND_DEGRADED_MIN_DIST_RATIO);
    readOptionalParam(fsSettings, "frontend_gradient_grid", FRONTEND_GRADIENT_GRID);
    readOptionalParam(fsSettings, "frontend_gradient_min", FRONTEND_GRADIENT_MIN);
    readOptionalParam(fsSettings, "frontend_fb_threshold", FRONTEND_FB_THRESHOLD);
    readOptionalParam(fsSettings, "frontend_lk_max_error", FRONTEND_LK_MAX_ERROR);
    readOptionalParam(fsSettings, "frontend_track_min_eigen", FRONTEND_TRACK_MIN_EIGEN);
    readOptionalParam(fsSettings, "frontend_ransac", FRONTEND_RANSAC);
    readOptionalParam(fsSettings, "frontend_ransac_strict", FRONTEND_RANSAC_STRICT);
    readOptionalParam(fsSettings, "frontend_ransac_min_points", FRONTEND_RANSAC_MIN_POINTS);
    readOptionalParam(fsSettings, "frontend_ransac_min_inliers", FRONTEND_RANSAC_MIN_INLIERS);
    readOptionalParam(fsSettings, "frontend_ransac_min_ratio", FRONTEND_RANSAC_MIN_RATIO);
    readOptionalParam(fsSettings, "frontend_quality_grid_cols", FRONTEND_QUALITY_GRID_COLS);
    readOptionalParam(fsSettings, "frontend_quality_grid_rows", FRONTEND_QUALITY_GRID_ROWS);
    readOptionalParam(fsSettings, "frontend_quality_min_tracked", FRONTEND_QUALITY_MIN_TRACKED);
    readOptionalParam(fsSettings, "frontend_min_quality_points", FRONTEND_MIN_QUALITY_POINTS);
    readOptionalParam(fsSettings, "frontend_min_lk_keep_ratio", FRONTEND_MIN_LK_KEEP_RATIO);
    readOptionalParam(fsSettings, "frontend_min_coverage_ratio", FRONTEND_MIN_COVERAGE_RATIO);
    readOptionalParam(fsSettings, "frontend_quality_min_eigen", FRONTEND_QUALITY_MIN_EIGEN);

    MULTIPLE_THREAD = fsSettings["multiple_thread"];
    readOptionalParam(fsSettings, "opencv_num_threads", OPENCV_NUM_THREADS);
    readOptionalParam(fsSettings, "image_sync_sleep_ms", IMAGE_SYNC_SLEEP_MS);
    OPENCV_NUM_THREADS = std::max(0, OPENCV_NUM_THREADS);
    IMAGE_SYNC_SLEEP_MS = std::max(1, IMAGE_SYNC_SLEEP_MS);

    USE_IMU = fsSettings["imu"];
    printf("USE_IMU: %d\n", USE_IMU);
    if(USE_IMU)
    {
        fsSettings["imu_topic"] >> IMU_TOPIC;
        printf("IMU_TOPIC: %s\n", IMU_TOPIC.c_str());
        ACC_N = fsSettings["acc_n"];
        ACC_W = fsSettings["acc_w"];
        GYR_N = fsSettings["gyr_n"];
        GYR_W = fsSettings["gyr_w"];
        G.z() = fsSettings["g_norm"];
    }

    SOLVER_TIME = fsSettings["max_solver_time"];
    NUM_ITERATIONS = fsSettings["max_num_iterations"];
    readOptionalParam(fsSettings, "marginalization_num_threads", MARGINALIZATION_NUM_THREADS);
    MARGINALIZATION_NUM_THREADS = std::max(1, MARGINALIZATION_NUM_THREADS);
    MIN_PARALLAX = fsSettings["keyframe_parallax"];
    MIN_PARALLAX = MIN_PARALLAX / FOCAL_LENGTH;

    fsSettings["output_path"] >> OUTPUT_FOLDER;
    if (!createDirectoryRecursive(OUTPUT_FOLDER))
        ROS_WARN("Failed to create output_path: %s", OUTPUT_FOLDER.c_str());
    VINS_RESULT_PATH = OUTPUT_FOLDER + "/stamped_traj_estimate.txt";

    std::cout << "result path " << VINS_RESULT_PATH << std::endl;
    std::ofstream foutC(VINS_RESULT_PATH, std::ios::out);
    if (!foutC || foutC.bad() || foutC.fail())
        std::cout << "VINS_RESULT_PATH not opened! Check if the path exists." << std::endl;
    foutC.close();

    ESTIMATE_EXTRINSIC = fsSettings["estimate_extrinsic"];
    if (ESTIMATE_EXTRINSIC == 2)
    {
        ROS_WARN("have no prior about extrinsic param, calibrate extrinsic param");
        RIC.push_back(Eigen::Matrix3d::Identity());
        TIC.push_back(Eigen::Vector3d::Zero());
        EX_CALIB_RESULT_PATH = OUTPUT_FOLDER + "/extrinsic_parameter.txt";
    }
    else 
    {
        if ( ESTIMATE_EXTRINSIC == 1)
        {
            ROS_WARN(" Optimize extrinsic param around initial guess!");
            EX_CALIB_RESULT_PATH = OUTPUT_FOLDER + "/extrinsic_parameter.txt";
        }
        if (ESTIMATE_EXTRINSIC == 0)
            ROS_WARN(" fix extrinsic param ");

        cv::Mat cv_T;
        fsSettings["body_T_cam0"] >> cv_T;
        Eigen::Matrix4d T;
        cv::cv2eigen(cv_T, T);
        RIC.push_back(T.block<3, 3>(0, 0));
        TIC.push_back(T.block<3, 1>(0, 3));
    } 
    
    NUM_OF_CAM = fsSettings["num_of_cam"];
    printf("camera number %d\n", NUM_OF_CAM);

    if(NUM_OF_CAM != 1 && NUM_OF_CAM != 2)
    {
        printf("num_of_cam should be 1 or 2\n");
        assert(0);
    }


    int pn = config_file.find_last_of('/');
    std::string configPath = config_file.substr(0, pn);
    
    std::string cam0Calib;
    fsSettings["cam0_calib"] >> cam0Calib;
    std::string cam0Path = configPath + "/" + cam0Calib;
    CAM_NAMES.push_back(cam0Path);

    if(NUM_OF_CAM == 2)
    {
        STEREO = 1;
        std::string cam1Calib;
        fsSettings["cam1_calib"] >> cam1Calib;
        std::string cam1Path = configPath + "/" + cam1Calib; 
        //printf("%s cam1 path\n", cam1Path.c_str() );
        CAM_NAMES.push_back(cam1Path);
        
        cv::Mat cv_T;
        fsSettings["body_T_cam1"] >> cv_T;
        Eigen::Matrix4d T;
        cv::cv2eigen(cv_T, T);
        RIC.push_back(T.block<3, 3>(0, 0));
        TIC.push_back(T.block<3, 1>(0, 3));
    }

    INIT_DEPTH = 5.0;
    BIAS_ACC_THRESHOLD = 0.1;
    BIAS_GYR_THRESHOLD = 0.1;

    TD = fsSettings["td"];
    ESTIMATE_TD = fsSettings["estimate_td"];
    if (ESTIMATE_TD)
        ROS_INFO_STREAM("Unsynchronized sensors, online estimate time offset, initial td: " << TD);
    else
        ROS_INFO_STREAM("Synchronized sensors, fix time offset: " << TD);

    ROW = fsSettings["image_height"];
    COL = fsSettings["image_width"];
    ROS_INFO("ROW: %d COL: %d ", ROW, COL);

    if(!USE_IMU)
    {
        ESTIMATE_EXTRINSIC = 0;
        ESTIMATE_TD = 0;
        printf("no imu, fix extrinsic param; no time offset calibration\n");
    }

    ROS_INFO_STREAM("Frontend config summary: max_cnt=" << MAX_CNT
                    << " min_dist=" << MIN_DIST
                    << " fb=" << FLOW_BACK
                    << " fb_thresh=" << FRONTEND_FB_THRESHOLD
                    << " feature_log_enable=" << FEATURE_LOG_ENABLE
                    << " feature_log_path=" << FEATURE_LOG_PATH);
    ROS_INFO_STREAM("Estimator config summary: estimate_extrinsic=" << ESTIMATE_EXTRINSIC
                    << " estimate_td=" << ESTIMATE_TD
                    << " blind_enable=" << BLIND_ENABLE
                    << " td=" << TD);

    fsSettings.release();
}
