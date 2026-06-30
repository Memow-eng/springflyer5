/*******************************************************
 * Copyright (C) 2019, Aerial Robotics Group, Hong Kong University of Science and Technology
 * 
 * This file is part of VINS.
 * 
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *******************************************************/

#include "estimator.h"
#include "../utility/visualization.h"
#include <algorithm>
#include <cmath>
#include <limits>

namespace
{
constexpr double kInvDepthMin = 1.0 / 100.0;
constexpr double kInvDepthMax = 1.0 / 0.1;

double clampVisualWeight(double weight)
{
    return std::max(0.10, std::min(1.0, weight));
}

double clampUnit(double value)
{
    return std::max(0.0, std::min(1.0, value));
}

double safeMedian(std::vector<double> values)
{
    if (values.empty())
        return 0.0;
    const size_t mid = values.size() / 2;
    std::nth_element(values.begin(), values.begin() + mid, values.end());
    double median = values[mid];
    if (values.size() % 2 == 0)
    {
        std::nth_element(values.begin(), values.begin() + mid - 1, values.end());
        median = 0.5 * (median + values[mid - 1]);
    }
    return median;
}

VisualHealthSnapshot makeVisualHealthSnapshot(
    const map<int, vector<pair<int, FeatureObservation>>> &image,
    const FrontendQuality &frontend_quality,
    int visual_track_num,
    double visual_parallax)
{
    VisualHealthSnapshot snapshot;
    snapshot.image_points = static_cast<int>(image.size());
    snapshot.prev_points = frontend_quality.prev_points;
    snapshot.tracked_after_lk = frontend_quality.tracked_after_lk;
    snapshot.tracked_after_ransac = frontend_quality.tracked_after_ransac;
    snapshot.total_points = frontend_quality.total_points;
    snapshot.tracked_points = frontend_quality.prev_points > 0
                                  ? frontend_quality.tracked_after_lk
                                  : snapshot.image_points;
    snapshot.lk_keep_ratio = frontend_quality.lk_keep_ratio;
    snapshot.mean_track_eigen = frontend_quality.mean_track_eigen;
    snapshot.mean_pixel_flow = frontend_quality.mean_pixel_flow;
    snapshot.coverage_ratio = frontend_quality.coverage_ratio;
    snapshot.brightness_mean = frontend_quality.brightness_mean;
    snapshot.dark_ratio = frontend_quality.dark_ratio;
    snapshot.saturated_ratio = frontend_quality.saturated_ratio;
    snapshot.contrast_std = frontend_quality.contrast_std;
    snapshot.blur_score = frontend_quality.blur_score;
    snapshot.photometric_health = frontend_quality.photometric_health;
    snapshot.low_tracking_quality = frontend_quality.low_tracking_quality;
    snapshot.weak_texture = frontend_quality.weak_texture;
    snapshot.poor_distribution = frontend_quality.poor_distribution;
    snapshot.ransac_rejected = frontend_quality.ransac_rejected;
    snapshot.low_parallax = frontend_quality.low_parallax;
    snapshot.visual_track_num = visual_track_num;
    snapshot.visual_parallax = visual_parallax;
    return snapshot;
}

double safeLogDet(const Eigen::Matrix3d &info)
{
    const double det = std::max(1e-12, info.determinant());
    return std::log(det);
}

double sampleThrustAcc(const std::deque<std::pair<double, double>> &thrust_buf, double t)
{
    if (thrust_buf.empty())
        return -1.0;

    double best_dt = 1e9;
    double best_thr = -1.0;
    for (const auto &sample : thrust_buf)
    {
        const double dt = std::fabs(sample.first - t);
        if (dt < best_dt)
        {
            best_dt = dt;
            best_thr = sample.second;
        }
    }
    if (best_dt > 0.1 || best_thr < 0.0)
        return -1.0;

    const double hover = std::max(1e-3, BLIND_THRUST_HOVER_THROTTLE);
    return (BLIND_THRUST_HOVER_ACC / hover) * best_thr;
}

bool featureReadyForOptimization(const FeaturePerId &feature)
{
    return feature.feature_per_frame.size() >= 4 &&
           std::isfinite(feature.estimated_depth) &&
           feature.estimated_depth > 0.0;
}
}

Estimator::Estimator(): f_manager{Rs}
{
    ROS_INFO("init begins");
    initThreadFlag = false;
    latest_estimator_latency_ms_ = 0.0;
    clearState();
}

Estimator::~Estimator()
{
    if (MULTIPLE_THREAD)
    {
        processThread.join();
        printf("join thread \n");
    }
}

void Estimator::clearState()
{
    mProcess.lock();
    while(!accBuf.empty())
        accBuf.pop();
    while(!gyrBuf.empty())
        gyrBuf.pop();
    while(!featureBuf.empty())
        featureBuf.pop();

    prevTime = -1;
    curTime = 0;
    openExEstimation = 0;
    initP = Eigen::Vector3d(0, 0, 0);
    initR = Eigen::Matrix3d::Identity();
    inputImageCnt = 0;
    initFirstPoseFlag = false;
    visual_state = VISUAL_HEALTHY;
    pending_state_ = VISUAL_HEALTHY;
    pending_count_ = 0;
    visual_track_num = 0;
    visual_parallax = 0.0;
    blind_start_time = -1.0;
    blind_active = false;
    blind_anchor_valid = false;
    low_flow_lock_active_ = false;
    home_loop_origin_valid_ = false;
    home_loop_active_ = false;
    last_state_valid_ = false;
    soft_failure_count_ = 0;
    bias_failure_count_ = 0;
    little_feature_count_ = 0;
    nonlinear_frame_count_ = 0;
    stereo_init_ready_reject_count_ = 0;
    home_loop_static_count_ = 0;
    home_loop_max_radius_ = 0.0;
    nonlinear_start_time_ = -1.0;
    low_flow_lock_P_.setZero();
    home_loop_origin_.setZero();
    blind_ba0.setZero();
    blind_bg0.setZero();
    blind_acc_body0.setZero();
    blind_v0.setZero();
    {
        std::lock_guard<std::mutex> lock(mThrust);
        thrustBuf.clear();
    }

    for (int i = 0; i < WINDOW_SIZE + 1; i++)
    {
        Rs[i].setIdentity();
        Ps[i].setZero();
        Vs[i].setZero();
        Bas[i].setZero();
        Bgs[i].setZero();
        dt_buf[i].clear();
        linear_acceleration_buf[i].clear();
        angular_velocity_buf[i].clear();

        if (pre_integrations[i] != nullptr)
        {
            delete pre_integrations[i];
        }
        pre_integrations[i] = nullptr;
    }

    for (int i = 0; i < NUM_OF_CAM; i++)
    {
        tic[i] = Vector3d::Zero();
        ric[i] = Matrix3d::Identity();
    }

    first_imu = false,
    sum_of_back = 0;
    sum_of_front = 0;
    frame_count = 0;
    solver_flag = INITIAL;
    initial_timestamp = 0;
    last_R = Matrix3d::Identity();
    last_P = Vector3d::Zero();
    last_R0 = Matrix3d::Identity();
    last_P0 = Vector3d::Zero();
    all_image_frame.clear();

    if (tmp_pre_integration != nullptr)
        delete tmp_pre_integration;
    if (last_marginalization_info != nullptr)
        delete last_marginalization_info;

    tmp_pre_integration = nullptr;
    last_marginalization_info = nullptr;
    last_marginalization_parameter_blocks.clear();

    f_manager.clearState();

    failure_occur = 0;
    latest_feature_quality_metrics_ = {1.0, 0.0, 1.0, 0.0};

    mProcess.unlock();
}

void Estimator::setParameter()
{
    mProcess.lock();
    for (int i = 0; i < NUM_OF_CAM; i++)
    {
        tic[i] = TIC[i];
        ric[i] = RIC[i];
        cout << " exitrinsic cam " << i << endl  << ric[i] << endl << tic[i].transpose() << endl;
    }
    f_manager.setRic(ric);
    ProjectionTwoFrameOneCamFactor::sqrt_info = FOCAL_LENGTH / 1.5 * Matrix2d::Identity();
    ProjectionTwoFrameTwoCamFactor::sqrt_info = FOCAL_LENGTH / 1.5 * Matrix2d::Identity();
    ProjectionOneFrameTwoCamFactor::sqrt_info = FOCAL_LENGTH / 1.5 * Matrix2d::Identity();
    td = TD;
    g = G;
    cout << "set g " << g.transpose() << endl;
    featureTracker.readIntrinsicParameter(CAM_NAMES);

    std::cout << "MULTIPLE_THREAD is " << MULTIPLE_THREAD << '\n';
    if (MULTIPLE_THREAD && !initThreadFlag)
    {
        initThreadFlag = true;
        processThread = std::thread(&Estimator::processMeasurements, this);
    }
    mProcess.unlock();
}

void Estimator::changeSensorType(int use_imu, int use_stereo)
{
    bool restart = false;
    mProcess.lock();
    if(!use_imu && !use_stereo)
        printf("at least use two sensors! \n");
    else
    {
        if(USE_IMU != use_imu)
        {
            USE_IMU = use_imu;
            if(USE_IMU)
            {
                // reuse imu; restart system
                restart = true;
            }
            else
            {
                if (last_marginalization_info != nullptr)
                    delete last_marginalization_info;

                tmp_pre_integration = nullptr;
                last_marginalization_info = nullptr;
                last_marginalization_parameter_blocks.clear();
            }
        }
        
        STEREO = use_stereo;
        printf("use imu %d use stereo %d\n", USE_IMU, STEREO);
    }
    mProcess.unlock();
    if(restart)
    {
        clearState();
        setParameter();
    }
}

void Estimator::inputImage(double t, const cv::Mat &_img, const cv::Mat &_img1)
{
    inputImageCnt++;
    map<int, vector<pair<int, FeatureObservation>>> featureFrame;
    FrontendQuality frontendQuality;
    TicToc featureTrackerTime;

    {
        std::lock_guard<std::mutex> lock(mTracker);
        if (solver_flag == NON_LINEAR && frame_count >= 1)
        {
            const Eigen::Matrix3d R_cur_prev_cam =
                ric[0].transpose() * Rs[frame_count].transpose() * Rs[frame_count - 1] * ric[0];
            featureTracker.setRelativeRotation(R_cur_prev_cam);
        }
        if(_img1.empty())
            featureFrame = featureTracker.trackImage(t, _img);
        else
            featureFrame = featureTracker.trackImage(t, _img, _img1);
        frontendQuality = featureTracker.getLastFrontendQuality();
    }
    //printf("featureTracker time: %f\n", featureTrackerTime.toc());

    if (SHOW_TRACK)
    {
        std::lock_guard<std::mutex> lock(mTracker);
        cv::Mat imgTrack = featureTracker.getTrackImage();
        pubTrackImage(imgTrack, t);
    }
    
    if(MULTIPLE_THREAD)  
    {     
        if(inputImageCnt % 2 == 0)
        {
            mBuf.lock();
            featureBuf.push(FeatureMeasurement(t, featureFrame, frontendQuality));
            mBuf.unlock();
        }
    }
    else
    {
        mBuf.lock();
        featureBuf.push(FeatureMeasurement(t, featureFrame, frontendQuality));
        mBuf.unlock();
        TicToc processTime;
        processMeasurements();
        printf("process time: %f\n", processTime.toc());
    }
    
}

void Estimator::inputIMU(double t, const Vector3d &linearAcceleration, const Vector3d &angularVelocity)
{
    mBuf.lock();
    accBuf.push(make_pair(t, linearAcceleration));
    gyrBuf.push(make_pair(t, angularVelocity));
    //printf("input imu with time %f \n", t);
    mBuf.unlock();

    if (!std::isfinite(latest_time) || latest_time <= 0.0)
        latest_time = t;

    if (solver_flag == NON_LINEAR)
    {
        mPropagate.lock();
        fastPredictIMU(t, linearAcceleration, angularVelocity);
        if (isLowFlowStationary() && low_flow_lock_active_ && low_flow_lock_P_.allFinite())
        {
            latest_P = low_flow_lock_P_;
            latest_V.setZero();
        }
        if (allowImuPropagateOutput())
            pubLatestOdometry(latest_P, latest_Q, latest_V, t);
        mPropagate.unlock();
    }
}

void Estimator::inputThrust(double t, double thrust_norm)
{
    std::lock_guard<std::mutex> lock(mThrust);
    thrustBuf.push_back(std::make_pair(t, thrust_norm));
    while (!thrustBuf.empty() && thrustBuf.front().first < t - 2.0)
        thrustBuf.pop_front();
}

double Estimator::getThrustAcc(double t)
{
    std::lock_guard<std::mutex> lock(mThrust);
    if (thrustBuf.empty())
        return -1.0;

    double best_dt = 1e9;
    double best_thr = -1.0;
    for (const auto &e : thrustBuf)
    {
        double d = std::fabs(e.first - t);
        if (d < best_dt)
        {
            best_dt = d;
            best_thr = e.second;
        }
    }
    if (best_dt > 0.1 || best_thr < 0.0)
        return -1.0;

    const double hover = std::max(1e-3, BLIND_THRUST_HOVER_THROTTLE);
    return (BLIND_THRUST_HOVER_ACC / hover) * best_thr;
}

void Estimator::inputFeature(double t, const map<int, vector<pair<int, FeatureObservation>>> &featureFrame)
{
    FrontendQuality frontendQuality;
    frontendQuality.total_points = static_cast<int>(featureFrame.size());
    frontendQuality.tracked_after_lk = frontendQuality.total_points;
    frontendQuality.tracked_after_ransac = frontendQuality.total_points;
    latest_frontend_quality_ = frontendQuality;
    mBuf.lock();
    featureBuf.push(FeatureMeasurement(t, featureFrame, frontendQuality));
    mBuf.unlock();

    if(!MULTIPLE_THREAD)
        processMeasurements();
}


bool Estimator::getIMUInterval(double t0, double t1, vector<pair<double, Eigen::Vector3d>> &accVector, 
                                vector<pair<double, Eigen::Vector3d>> &gyrVector)
{
    if(accBuf.empty())
    {
        printf("not receive imu\n");
        return false;
    }
    //printf("get imu from %f %f\n", t0, t1);
    //printf("imu fornt time %f   imu end time %f\n", accBuf.front().first, accBuf.back().first);
    if(t1 <= accBuf.back().first)
    {
        while (accBuf.front().first <= t0)
        {
            accBuf.pop();
            gyrBuf.pop();
        }
        while (accBuf.front().first < t1)
        {
            accVector.push_back(accBuf.front());
            accBuf.pop();
            gyrVector.push_back(gyrBuf.front());
            gyrBuf.pop();
        }
        accVector.push_back(accBuf.front());
        gyrVector.push_back(gyrBuf.front());
    }
    else
    {
        printf("wait for imu\n");
        return false;
    }
    return true;
}

bool Estimator::IMUAvailable(double t)
{
    if(!accBuf.empty() && t <= accBuf.back().first)
        return true;
    else
        return false;
}

void Estimator::processMeasurements()
{
    while (1)
    {
        //printf("process measurments\n");
        FeatureMeasurement feature;
        vector<pair<double, Eigen::Vector3d>> accVector, gyrVector;
        if(!featureBuf.empty())
        {
            feature = featureBuf.front();
            curTime = feature.t + td;
            while(1)
            {
                if ((!USE_IMU  || IMUAvailable(feature.t + td)))
                    break;
                else
                {
                    printf("wait for imu ... \n");
                    if (! MULTIPLE_THREAD)
                        return;
                    std::chrono::milliseconds dura(5);
                    std::this_thread::sleep_for(dura);
                }
            }
            mBuf.lock();
            if(USE_IMU)
                getIMUInterval(prevTime, curTime, accVector, gyrVector);

            featureBuf.pop();
            mBuf.unlock();

            if(USE_IMU)
            {
                if(!initFirstPoseFlag)
                    initFirstIMUPose(accVector);
                for(size_t i = 0; i < accVector.size(); i++)
                {
                    double dt;
                    if(i == 0)
                    {
                        if (prevTime < 0.0 || !std::isfinite(prevTime))
                            dt = 0.0;
                        else
                            dt = accVector[i].first - prevTime;
                    }
                    else if (i == accVector.size() - 1)
                        dt = curTime - accVector[i - 1].first;
                    else
                        dt = accVector[i].first - accVector[i - 1].first;
                    processIMU(accVector[i].first, dt, accVector[i].second, gyrVector[i].second);
                }
            }
            mProcess.lock();
            processImage(feature.features, feature.t, feature.quality);
            prevTime = curTime;

            printStatistics(*this, 0);

            std_msgs::Header header;
            header.frame_id = "world";
            header.stamp = ros::Time(feature.t);

            pubOdometry(*this, header);
            pubKeyPoses(*this, header);
            pubCameraPose(*this, header);
            pubPointCloud(*this, header);
            pubKeyframe(*this);
            pubTF(*this, header);
            mProcess.unlock();
        }

        if (! MULTIPLE_THREAD)
            break;

        std::chrono::milliseconds dura(2);
        std::this_thread::sleep_for(dura);
    }
}


void Estimator::initFirstIMUPose(vector<pair<double, Eigen::Vector3d>> &accVector)
{
    printf("init first imu pose\n");
    initFirstPoseFlag = true;
    //return;
    Eigen::Vector3d averAcc(0, 0, 0);
    int n = (int)accVector.size();
    for(size_t i = 0; i < accVector.size(); i++)
    {
        averAcc = averAcc + accVector[i].second;
    }
    averAcc = averAcc / n;
    printf("averge acc %f %f %f\n", averAcc.x(), averAcc.y(), averAcc.z());
    Matrix3d R0 = Utility::g2R(averAcc);
    double yaw = Utility::R2ypr(R0).x();
    R0 = Utility::ypr2R(Eigen::Vector3d{-yaw, 0, 0}) * R0;
    Rs[0] = R0;
    cout << "init R0 " << endl << Rs[0] << endl;
    //Vs[0] = Vector3d(5, 0, 0);
}

void Estimator::initFirstPose(Eigen::Vector3d p, Eigen::Matrix3d r)
{
    Ps[0] = p;
    Rs[0] = r;
    initP = p;
    initR = r;
}


void Estimator::processIMU(double t, double dt, const Vector3d &linear_acceleration, const Vector3d &angular_velocity)
{
    if (!first_imu)
    {
        first_imu = true;
        acc_0 = linear_acceleration;
        gyr_0 = angular_velocity;
    }

    if (!std::isfinite(dt) || dt <= 0.0 || dt > 0.05)
    {
        ROS_WARN_THROTTLE(1.0, "skip abnormal IMU dt: %.6f at t=%.6f", dt, t);
        acc_0 = linear_acceleration;
        gyr_0 = angular_velocity;
        return;
    }

    if (!pre_integrations[frame_count])
    {
        pre_integrations[frame_count] = new IntegrationBase{acc_0, gyr_0, Bas[frame_count], Bgs[frame_count]};
    }
    if (frame_count != 0)
    {
        pre_integrations[frame_count]->push_back(dt, linear_acceleration, angular_velocity);
        //if(solver_flag != NON_LINEAR)
            tmp_pre_integration->push_back(dt, linear_acceleration, angular_velocity);

        dt_buf[frame_count].push_back(dt);
        linear_acceleration_buf[frame_count].push_back(linear_acceleration);
        angular_velocity_buf[frame_count].push_back(angular_velocity);

        int j = frame_count;         
        Vector3d un_acc_0 = Rs[j] * (acc_0 - Bas[j]) - g;
        Vector3d un_gyr = 0.5 * (gyr_0 + angular_velocity) - Bgs[j];
        Rs[j] *= Utility::deltaQ(un_gyr * dt).toRotationMatrix();
        Vector3d un_acc_1 = Rs[j] * (linear_acceleration - Bas[j]) - g;
        Vector3d un_acc = 0.5 * (un_acc_0 + un_acc_1);
        Ps[j] += dt * Vs[j] + 0.5 * dt * dt * un_acc;
        Vs[j] += dt * un_acc;
    }
    acc_0 = linear_acceleration;
    gyr_0 = angular_velocity; 
}

void Estimator::processImage(const map<int, vector<pair<int, FeatureObservation>>> &image,
                             const double header,
                             const FrontendQuality &frontend_quality)
{
    ROS_DEBUG("new image coming ------------------------------------------");
    ROS_DEBUG("Adding feature points %lu", image.size());
    latest_frontend_quality_ = frontend_quality;
    const bool skip_visual_frame = shouldSkipVisualFrame(image, frontend_quality);
    if (skip_visual_frame)
        ROS_DEBUG("visual frame kept with soft weights despite degraded frontend: %lu points", image.size());

    if (f_manager.addFeatureCheckParallax(frame_count, image, td))
    {
        marginalization_flag = MARGIN_OLD;
        //printf("keyframe\n");
    }
    else
    {
        marginalization_flag = MARGIN_SECOND_NEW;
        //printf("non-keyframe\n");
    }
    updateVisualHealth(image, frontend_quality, header);

    ROS_DEBUG("%s", marginalization_flag ? "Non-keyframe" : "Keyframe");
    ROS_DEBUG("Solving %d", frame_count);
    ROS_DEBUG("number of feature: %d", f_manager.getFeatureCount());
    Headers[frame_count] = header;

    ImageFrame imageframe(image, header);
    imageframe.pre_integration = tmp_pre_integration;
    all_image_frame.insert(make_pair(header, imageframe));
    tmp_pre_integration = new IntegrationBase{acc_0, gyr_0, Bas[frame_count], Bgs[frame_count]};

    if(ESTIMATE_EXTRINSIC == 2)
    {
        ROS_INFO("calibrating extrinsic param, rotation movement is needed");
        if (frame_count != 0)
        {
            vector<pair<Vector3d, Vector3d>> corres = f_manager.getCorresponding(frame_count - 1, frame_count);
            Matrix3d calib_ric;
            if (initial_ex_rotation.CalibrationExRotation(corres, pre_integrations[frame_count]->delta_q, calib_ric))
            {
                ROS_WARN("initial extrinsic rotation calib success");
                ROS_WARN_STREAM("initial extrinsic rotation: " << endl << calib_ric);
                ric[0] = calib_ric;
                RIC[0] = calib_ric;
                ESTIMATE_EXTRINSIC = 1;
            }
        }
    }

    if (solver_flag == INITIAL)
    {
        // monocular + IMU initilization
        if (!STEREO && USE_IMU)
        {
            if (frame_count == WINDOW_SIZE)
            {
                bool result = false;
                if(ESTIMATE_EXTRINSIC != 2 && (header - initial_timestamp) > 0.1)
                {
                    result = initialStructure();
                    initial_timestamp = header;   
                }
                if(result)
                {
                    optimization();
                    updateLatestStates();
                    solver_flag = NON_LINEAR;
                    nonlinear_start_time_ = header;
                    nonlinear_frame_count_ = 0;
                    bias_failure_count_ = 0;
                    slideWindow();
                    ROS_INFO("Initialization finish!");
                }
                else
                    slideWindow();
            }
        }

        // stereo + IMU initilization
        if(STEREO && USE_IMU)
        {
            f_manager.initFramePoseByPnP(frame_count, Ps, Rs, tic, ric);
            f_manager.triangulate(frame_count, Ps, Rs, tic, ric);
            if (frame_count == WINDOW_SIZE)
            {
                map<double, ImageFrame>::iterator frame_it;
                int i = 0;
                for (frame_it = all_image_frame.begin(); frame_it != all_image_frame.end(); frame_it++)
                {
                    frame_it->second.R = Rs[i];
                    frame_it->second.T = Ps[i];
                    i++;
                }
                if (!isStereoInitializationReady(header))
                {
                    ++stereo_init_ready_reject_count_;
                    ROS_WARN("VINS stereo init ready gate slide retry %d at %.3f",
                             stereo_init_ready_reject_count_, header);
                    slideInitializationCandidate(header);
                    return;
                }
                if (!solveGyroscopeBias(all_image_frame, Bgs))
                {
                    resetInitializationCandidate(header);
                    return;
                }
                for (int i = 0; i <= WINDOW_SIZE; i++)
                {
                    if (pre_integrations[i] != nullptr)
                        pre_integrations[i]->repropagate(Vector3d::Zero(), Bgs[i]);
                }
                optimization();
                updateLatestStates();
                logStereoInitializationQuality(header);
                if (!isStereoInitializationSane(header))
                {
                    resetInitializationCandidate(header);
                    return;
                }
                solver_flag = NON_LINEAR;
                nonlinear_start_time_ = header;
                nonlinear_frame_count_ = 0;
                bias_failure_count_ = 0;
                stereo_init_ready_reject_count_ = 0;
                slideWindow();
                ROS_INFO("Initialization finish!");
            }
        }

        // stereo only initilization
        if(STEREO && !USE_IMU)
        {
            f_manager.initFramePoseByPnP(frame_count, Ps, Rs, tic, ric);
            f_manager.triangulate(frame_count, Ps, Rs, tic, ric);
            optimization();

            if(frame_count == WINDOW_SIZE)
            {
                optimization();
                updateLatestStates();
                solver_flag = NON_LINEAR;
                nonlinear_start_time_ = header;
                nonlinear_frame_count_ = 0;
                bias_failure_count_ = 0;
                slideWindow();
                ROS_INFO("Initialization finish!");
            }
        }

        if(frame_count < WINDOW_SIZE)
        {
            frame_count++;
            int prev_frame = frame_count - 1;
            Ps[frame_count] = Ps[prev_frame];
            Vs[frame_count] = Vs[prev_frame];
            Rs[frame_count] = Rs[prev_frame];
            Bas[frame_count] = Bas[prev_frame];
            Bgs[frame_count] = Bgs[prev_frame];
        }

    }
    else
    {
        TicToc t_solve;
        if(!USE_IMU)
            f_manager.initFramePoseByPnP(frame_count, Ps, Rs, tic, ric);
        f_manager.triangulate(frame_count, Ps, Rs, tic, ric);
        optimization();
        latest_estimator_latency_ms_ = std::max(0.0, (ros::Time::now().toSec() - header) * 1000.0);
        set<int> removeIndex;
        if (!isBlind())
        {
            outliersRejection(removeIndex);
            f_manager.removeOutlier(removeIndex);
        }
        if (! MULTIPLE_THREAD)
        {
            std::lock_guard<std::mutex> lock(mTracker);
            featureTracker.removeOutliers(removeIndex);
            predictPtsInNextFrame();
        }
        else
        {
            std::lock_guard<std::mutex> lock(mTracker);
            predictPtsInNextFrame();
        }
        ++nonlinear_frame_count_;
            
        ROS_DEBUG("solver costs: %fms", t_solve.toc());

        if (failureDetection())
        {
            ROS_WARN("failure detection!");
            failure_occur = 1;
            clearState();
            setParameter();
            ROS_WARN("system reboot!");
            return;
        }

        slideWindow();
        f_manager.removeFailures();
        // prepare output of VINS
        key_poses.clear();
        for (int i = 0; i <= WINDOW_SIZE; i++)
            key_poses.push_back(Ps[i]);

        last_R = Rs[WINDOW_SIZE];
        last_P = Ps[WINDOW_SIZE];
        last_R0 = Rs[0];
        last_P0 = Ps[0];
        last_state_valid_ = true;
        updateLatestStates();
    }  
}

void Estimator::logStereoInitializationGateStats(double header, const char *tag) const
{
    int stereo_obs = 0;
    int long_tracks = 0;
    int valid_depth = 0;
    for (const auto &it_per_id : f_manager.feature)
    {
        bool has_stereo = false;
        for (const auto &it_per_frame : it_per_id.feature_per_frame)
        {
            if (it_per_frame.is_stereo)
            {
                has_stereo = true;
                stereo_obs++;
            }
        }
        if (has_stereo && static_cast<int>(it_per_id.feature_per_frame.size()) >= 4)
            long_tracks++;
        if (it_per_id.estimated_depth > 0.0 && it_per_id.estimated_depth < 20.0)
            valid_depth++;
    }

    int low_dynamic_frames = 0;
    double max_gyr = 0.0;
    double max_acc_dev = 0.0;
    double sum_gyr2 = 0.0;
    int gyr_count = 0;
    Vector3d acc_mean = Vector3d::Zero();
    int acc_count = 0;
    for (int i = 0; i <= frame_count; i++)
    {
        if (isLowDynamic(i))
            low_dynamic_frames++;

        for (size_t k = 0; k < angular_velocity_buf[i].size(); k++)
        {
            max_gyr = std::max(max_gyr, angular_velocity_buf[i][k].norm());
            sum_gyr2 += angular_velocity_buf[i][k].squaredNorm();
            gyr_count++;
        }
        for (size_t k = 0; k < linear_acceleration_buf[i].size(); k++)
        {
            max_acc_dev = std::max(max_acc_dev, fabs(linear_acceleration_buf[i][k].norm() - G.norm()));
            acc_mean += linear_acceleration_buf[i][k];
            acc_count++;
        }
    }
    if (acc_count > 0)
        acc_mean /= static_cast<double>(acc_count);

    double acc_excitation2 = 0.0;
    for (int i = 0; i <= frame_count; i++)
    {
        for (size_t k = 0; k < linear_acceleration_buf[i].size(); k++)
            acc_excitation2 += (linear_acceleration_buf[i][k] - acc_mean).squaredNorm();
    }
    const double acc_excitation_rms = acc_count > 0 ? std::sqrt(acc_excitation2 / static_cast<double>(acc_count)) : 0.0;
    const double gyr_rms = gyr_count > 0 ? std::sqrt(sum_gyr2 / static_cast<double>(gyr_count)) : 0.0;
    double preint_motion = 0.0;
    double preint_motion_sum = 0.0;
    for (int i = 1; i <= frame_count; i++)
    {
        if (pre_integrations[i] == nullptr)
            continue;
        const double alpha_norm = pre_integrations[i]->delta_p.norm();
        preint_motion = std::max(preint_motion, alpha_norm);
        preint_motion_sum += alpha_norm;
    }

    const int motion_frames = frame_count + 1 - low_dynamic_frames;
    const double low_dynamic_ratio = (frame_count + 1) > 0 ?
        static_cast<double>(low_dynamic_frames) / static_cast<double>(frame_count + 1) : 1.0;
    const double long_track_ratio = f_manager.last_track_num > 0 ?
        static_cast<double>(f_manager.long_track_num) / static_cast<double>(f_manager.last_track_num) : 0.0;

    ROS_WARN("VINS stereo init gate stats %s at %.3f: motion_frames=%d tracks=%d long=%d stereo_obs=%d valid_depth=%d par=%.2f lowdyn=%d/%d lowdyn_ratio=%.2f long_ratio=%.2f max_gyr=%.2f gyr_rms=%.3f max_acc_dev=%.2f acc_exc_rms=%.3f preint_max=%.5f preint_sum=%.5f",
             tag, header, motion_frames, f_manager.last_track_num, f_manager.long_track_num,
             stereo_obs, valid_depth, f_manager.last_average_parallax,
             low_dynamic_frames, frame_count + 1, low_dynamic_ratio, long_track_ratio,
             max_gyr, gyr_rms, max_acc_dev, acc_excitation_rms,
             preint_motion, preint_motion_sum);
}

bool Estimator::isStereoInitializationReady(double header) const
{
    int stereo_obs = 0;
    int long_tracks = 0;
    int valid_depth = 0;
    for (const auto &it_per_id : f_manager.feature)
    {
        bool has_stereo = false;
        for (const auto &it_per_frame : it_per_id.feature_per_frame)
        {
            if (it_per_frame.is_stereo)
            {
                has_stereo = true;
                stereo_obs++;
            }
        }
        if (has_stereo && static_cast<int>(it_per_id.feature_per_frame.size()) >= 4)
            long_tracks++;
        if (it_per_id.estimated_depth > 0.0 && it_per_id.estimated_depth < 20.0)
            valid_depth++;
    }

    int low_dynamic_frames = 0;
    double max_gyr = 0.0;
    double max_acc_dev = 0.0;
    double sum_gyr2 = 0.0;
    int gyr_count = 0;
    Vector3d acc_mean = Vector3d::Zero();
    int acc_count = 0;
    for (int i = 0; i <= frame_count; i++)
    {
        if (isLowDynamic(i))
            low_dynamic_frames++;

        for (size_t k = 0; k < angular_velocity_buf[i].size(); k++)
        {
            max_gyr = std::max(max_gyr, angular_velocity_buf[i][k].norm());
            sum_gyr2 += angular_velocity_buf[i][k].squaredNorm();
            gyr_count++;
        }
        for (size_t k = 0; k < linear_acceleration_buf[i].size(); k++)
        {
            max_acc_dev = std::max(max_acc_dev, fabs(linear_acceleration_buf[i][k].norm() - G.norm()));
            acc_mean += linear_acceleration_buf[i][k];
            acc_count++;
        }
    }
    if (acc_count > 0)
        acc_mean /= static_cast<double>(acc_count);

    double acc_excitation2 = 0.0;
    for (int i = 0; i <= frame_count; i++)
    {
        for (size_t k = 0; k < linear_acceleration_buf[i].size(); k++)
            acc_excitation2 += (linear_acceleration_buf[i][k] - acc_mean).squaredNorm();
    }
    const double acc_excitation_rms = acc_count > 0 ? std::sqrt(acc_excitation2 / static_cast<double>(acc_count)) : 0.0;
    const double gyr_rms = gyr_count > 0 ? std::sqrt(sum_gyr2 / static_cast<double>(gyr_count)) : 0.0;
    double preint_motion = 0.0;
    double preint_motion_sum = 0.0;
    for (int i = 1; i <= frame_count; i++)
    {
        if (pre_integrations[i] == nullptr)
            continue;
        const double alpha_norm = pre_integrations[i]->delta_p.norm();
        preint_motion = std::max(preint_motion, alpha_norm);
        preint_motion_sum += alpha_norm;
    }

    const int motion_frames = frame_count + 1 - low_dynamic_frames;
    const double low_dynamic_ratio = (frame_count + 1) > 0 ?
        static_cast<double>(low_dynamic_frames) / static_cast<double>(frame_count + 1) : 1.0;
    const double long_track_ratio = f_manager.last_track_num > 0 ?
        static_cast<double>(f_manager.long_track_num) / static_cast<double>(f_manager.last_track_num) : 0.0;
    const bool not_enough_motion = motion_frames < 3;
    const bool low_parallax = f_manager.last_average_parallax < 1.0;
    const bool violent_motion = max_gyr > 0.35 || max_acc_dev > 0.8;
    const bool excessive_parallax = f_manager.last_average_parallax > 12.0 && !violent_motion &&
                                    max_gyr < 0.12 && max_acc_dev < 0.25;
    const bool front_end_healthy = f_manager.last_track_num >= 80 &&
                                   f_manager.long_track_num >= 50 &&
                                   long_tracks >= 50 &&
                                   stereo_obs >= 80 &&
                                   valid_depth >= 20;
    const bool escape_ready = stereo_init_ready_reject_count_ >= 30 && front_end_healthy &&
                              !not_enough_motion && !low_parallax && !excessive_parallax &&
                              !violent_motion;
    const bool low_acc_excitation = acc_excitation_rms < 0.12 && !escape_ready;
    const bool enough_tracks = escape_ready ||
                               (f_manager.last_track_num >= 100 &&
                                f_manager.long_track_num >= 80 &&
                                long_tracks >= 80);
    const bool enough_stereo = stereo_obs >= 80 && valid_depth >= 20;

    if (not_enough_motion || low_parallax || excessive_parallax ||
        violent_motion || low_acc_excitation || !enough_tracks || !enough_stereo)
    {
        ROS_WARN("VINS stereo init gate reject at %.3f: motion_frames=%d low_parallax=%d excessive_parallax=%d violent=%d low_acc_exc=%d escape=%d retry=%d tracks=%d long=%d stereo_obs=%d valid_depth=%d par=%.2f lowdyn=%d/%d lowdyn_ratio=%.2f long_ratio=%.2f max_gyr=%.2f gyr_rms=%.3f max_acc_dev=%.2f acc_exc_rms=%.3f preint_max=%.5f preint_sum=%.5f",
                 header, motion_frames, low_parallax, excessive_parallax, violent_motion,
                 low_acc_excitation, escape_ready, stereo_init_ready_reject_count_,
                 f_manager.last_track_num, f_manager.long_track_num,
                 stereo_obs, valid_depth, f_manager.last_average_parallax,
                 low_dynamic_frames, frame_count + 1, low_dynamic_ratio, long_track_ratio,
                 max_gyr, gyr_rms, max_acc_dev, acc_excitation_rms,
                 preint_motion, preint_motion_sum);
        return false;
    }

    ROS_WARN("VINS stereo init gate accept at %.3f: motion_frames=%d escape=%d retry=%d tracks=%d long=%d stereo_obs=%d valid_depth=%d par=%.2f lowdyn=%d/%d lowdyn_ratio=%.2f long_ratio=%.2f max_gyr=%.2f gyr_rms=%.3f max_acc_dev=%.2f acc_exc_rms=%.3f preint_max=%.5f preint_sum=%.5f",
             header, motion_frames, escape_ready, stereo_init_ready_reject_count_,
             f_manager.last_track_num, f_manager.long_track_num,
             stereo_obs, valid_depth, f_manager.last_average_parallax,
             low_dynamic_frames, frame_count + 1, low_dynamic_ratio, long_track_ratio,
             max_gyr, gyr_rms, max_acc_dev, acc_excitation_rms,
             preint_motion, preint_motion_sum);
    return true;
}

bool Estimator::isStereoInitializationSane(double header) const
{
    double max_speed = 0.0;
    double max_step = 0.0;
    double max_acc_bias = 0.0;
    double max_gyr_bias = 0.0;
    for (int i = 0; i <= frame_count; i++)
    {
        if (!Ps[i].allFinite() || !Vs[i].allFinite() || !Bas[i].allFinite() ||
            !Bgs[i].allFinite() || !Rs[i].allFinite())
        {
            ROS_WARN("VINS stereo init sanity reject at %.3f: non-finite state at frame %d", header, i);
            return false;
        }

        max_speed = std::max(max_speed, Vs[i].norm());
        max_acc_bias = std::max(max_acc_bias, Bas[i].norm());
        max_gyr_bias = std::max(max_gyr_bias, Bgs[i].norm());
        if (i > 0)
            max_step = std::max(max_step, (Ps[i] - Ps[i - 1]).norm());
    }

    const double window_motion = (Ps[frame_count] - Ps[0]).norm();
    if (max_speed > 8.0 || max_step > 2.0 || max_acc_bias > 1.5 || max_gyr_bias > 0.3)
    {
        ROS_WARN("VINS stereo init sanity reject at %.3f: max_speed=%.3f max_step=%.3f window_motion=%.3f ba=%.3f bg=%.3f",
                 header, max_speed, max_step, window_motion, max_acc_bias, max_gyr_bias);
        return false;
    }

    ROS_WARN("VINS stereo init sanity accept at %.3f: max_speed=%.3f max_step=%.3f window_motion=%.3f ba=%.3f bg=%.3f",
             header, max_speed, max_step, window_motion, max_acc_bias, max_gyr_bias);
    return true;
}

void Estimator::logStereoInitializationQuality(double header) const
{
    std::vector<double> reproj_errors;
    int valid_landmarks = 0;
    int visual_obs = 0;
    int stereo_obs = 0;
    for (const auto &it_per_id : f_manager.feature)
    {
        if (it_per_id.estimated_depth <= 0.0 || it_per_id.estimated_depth > 20.0 ||
            it_per_id.feature_per_frame.empty())
            continue;

        valid_landmarks++;
        const int imu_i = it_per_id.start_frame;
        if (imu_i < 0 || imu_i > frame_count)
            continue;

        const Vector3d pts_i = it_per_id.feature_per_frame[0].point;
        const double depth = it_per_id.estimated_depth;
        int imu_j = imu_i - 1;
        for (const auto &it_per_frame : it_per_id.feature_per_frame)
        {
            imu_j++;
            if (imu_j < 0 || imu_j > frame_count)
                continue;

            if (imu_i != imu_j)
            {
                const double err = reprojectionError(Rs[imu_i], Ps[imu_i], ric[0], tic[0],
                                                     Rs[imu_j], Ps[imu_j], ric[0], tic[0],
                                                     depth, pts_i, it_per_frame.point);
                if (std::isfinite(err))
                {
                    reproj_errors.push_back(err * FOCAL_LENGTH);
                    visual_obs++;
                }
            }

            if (it_per_frame.is_stereo)
            {
                const double err = reprojectionError(Rs[imu_i], Ps[imu_i], ric[0], tic[0],
                                                     Rs[imu_j], Ps[imu_j], ric[1], tic[1],
                                                     depth, pts_i, it_per_frame.pointRight);
                if (std::isfinite(err))
                {
                    reproj_errors.push_back(err * FOCAL_LENGTH);
                    stereo_obs++;
                }
            }
        }
    }

    std::vector<double> imu_rot_res;
    std::vector<double> imu_vel_res;
    std::vector<double> imu_pos_res;
    for (int i = 0; i < frame_count; i++)
    {
        const int j = i + 1;
        if (pre_integrations[j] == nullptr || pre_integrations[j]->sum_dt <= 0.0)
            continue;

        const double dt = pre_integrations[j]->sum_dt;
        const Matrix3d dp_dba = pre_integrations[j]->jacobian.template block<3, 3>(O_P, O_BA);
        const Matrix3d dp_dbg = pre_integrations[j]->jacobian.template block<3, 3>(O_P, O_BG);
        const Matrix3d dv_dba = pre_integrations[j]->jacobian.template block<3, 3>(O_V, O_BA);
        const Matrix3d dv_dbg = pre_integrations[j]->jacobian.template block<3, 3>(O_V, O_BG);
        const Matrix3d dq_dbg = pre_integrations[j]->jacobian.template block<3, 3>(O_R, O_BG);
        const Vector3d dba = Bas[i] - pre_integrations[j]->linearized_ba;
        const Vector3d dbg = Bgs[i] - pre_integrations[j]->linearized_bg;

        const Quaterniond corrected_delta_q =
            pre_integrations[j]->delta_q * Utility::deltaQ(dq_dbg * dbg);
        const Vector3d corrected_delta_v =
            pre_integrations[j]->delta_v + dv_dba * dba + dv_dbg * dbg;
        const Vector3d corrected_delta_p =
            pre_integrations[j]->delta_p + dp_dba * dba + dp_dbg * dbg;

        const Quaterniond Qi(Rs[i]);
        const Quaterniond Qj(Rs[j]);
        const Vector3d r_theta =
            2.0 * (corrected_delta_q.inverse() * (Qi.inverse() * Qj)).vec();
        const Vector3d r_beta =
            Qi.inverse() * (Vs[j] - Vs[i] - g * dt) - corrected_delta_v;
        const Vector3d r_alpha =
            Qi.inverse() * (Ps[j] - Ps[i] - Vs[i] * dt - 0.5 * g * dt * dt) - corrected_delta_p;

        imu_rot_res.push_back(r_theta.norm());
        imu_vel_res.push_back(r_beta.norm());
        imu_pos_res.push_back(r_alpha.norm());
    }

    auto percentile = [](std::vector<double> values, double q) -> double {
        if (values.empty())
            return -1.0;
        std::sort(values.begin(), values.end());
        const double pos = q * static_cast<double>(values.size() - 1);
        const size_t lo = static_cast<size_t>(std::floor(pos));
        const size_t hi = static_cast<size_t>(std::ceil(pos));
        if (lo == hi)
            return values[lo];
        return values[lo] * (static_cast<double>(hi) - pos) +
               values[hi] * (pos - static_cast<double>(lo));
    };

    ROS_WARN("VINS stereo init quality at %.3f: reproj_med_px=%.3f reproj_p90_px=%.3f valid_landmarks=%d visual_obs=%d stereo_obs=%d imu_rot_med=%.5f imu_vel_med=%.5f imu_pos_med=%.5f",
             header,
             percentile(reproj_errors, 0.5), percentile(reproj_errors, 0.9),
             valid_landmarks, visual_obs, stereo_obs,
             percentile(imu_rot_res, 0.5),
             percentile(imu_vel_res, 0.5),
             percentile(imu_pos_res, 0.5));
}

void Estimator::resetInitializationCandidate(double header)
{
    const Matrix3d current_R = Rs[frame_count];
    const Vector3d current_Ba = Bas[frame_count];
    const Vector3d current_Bg = Bgs[frame_count];

    for (auto &it : all_image_frame)
    {
        if (it.second.pre_integration != nullptr)
        {
            delete it.second.pre_integration;
            it.second.pre_integration = nullptr;
        }
    }
    all_image_frame.clear();

    for (int i = 0; i < WINDOW_SIZE + 1; i++)
    {
        if (pre_integrations[i] != nullptr)
            delete pre_integrations[i];
        pre_integrations[i] = nullptr;

        Headers[i] = 0.0;
        Ps[i].setZero();
        Vs[i].setZero();
        Rs[i] = current_R;
        Bas[i] = current_Ba;
        Bgs[i] = current_Bg;
        dt_buf[i].clear();
        linear_acceleration_buf[i].clear();
        angular_velocity_buf[i].clear();
    }

    if (tmp_pre_integration != nullptr)
        delete tmp_pre_integration;
    tmp_pre_integration = new IntegrationBase{acc_0, gyr_0, Bas[0], Bgs[0]};

    frame_count = 0;
    marginalization_flag = MARGIN_OLD;
    stereo_init_ready_reject_count_ = 0;
    initial_timestamp = header;
    f_manager.clearState();

    ROS_WARN("VINS stereo init candidate reset at %.3f", header);
}

void Estimator::slideInitializationCandidate(double header)
{
    if (frame_count != WINDOW_SIZE)
    {
        ROS_WARN("VINS stereo init candidate slide ignored at %.3f: frame_count=%d",
                 header, frame_count);
        return;
    }

    const double t_0 = Headers[0];
    back_R0 = Rs[0];
    back_P0 = Ps[0];

    for (int i = 0; i < WINDOW_SIZE; i++)
    {
        Headers[i] = Headers[i + 1];
        Rs[i].swap(Rs[i + 1]);
        Ps[i].swap(Ps[i + 1]);
        Vs[i].swap(Vs[i + 1]);
        Bas[i].swap(Bas[i + 1]);
        Bgs[i].swap(Bgs[i + 1]);
        std::swap(pre_integrations[i], pre_integrations[i + 1]);
        dt_buf[i].swap(dt_buf[i + 1]);
        linear_acceleration_buf[i].swap(linear_acceleration_buf[i + 1]);
        angular_velocity_buf[i].swap(angular_velocity_buf[i + 1]);
    }

    Headers[WINDOW_SIZE] = Headers[WINDOW_SIZE - 1];
    Ps[WINDOW_SIZE] = Ps[WINDOW_SIZE - 1];
    Rs[WINDOW_SIZE] = Rs[WINDOW_SIZE - 1];
    Vs[WINDOW_SIZE] = Vs[WINDOW_SIZE - 1];
    Bas[WINDOW_SIZE] = Bas[WINDOW_SIZE - 1];
    Bgs[WINDOW_SIZE] = Bgs[WINDOW_SIZE - 1];

    if (pre_integrations[WINDOW_SIZE] != nullptr)
        delete pre_integrations[WINDOW_SIZE];
    pre_integrations[WINDOW_SIZE] =
        new IntegrationBase{acc_0, gyr_0, Bas[WINDOW_SIZE], Bgs[WINDOW_SIZE]};
    dt_buf[WINDOW_SIZE].clear();
    linear_acceleration_buf[WINDOW_SIZE].clear();
    angular_velocity_buf[WINDOW_SIZE].clear();

    for (auto it = all_image_frame.begin(); it != all_image_frame.end();)
    {
        if (it->first > t_0 + 1e-9)
            break;
        if (it->second.pre_integration != nullptr)
        {
            delete it->second.pre_integration;
            it->second.pre_integration = nullptr;
        }
        it = all_image_frame.erase(it);
    }

    f_manager.removeBack();
    marginalization_flag = MARGIN_OLD;

    ROS_WARN("VINS stereo init candidate slide at %.3f: dropped %.3f retry=%d",
             header, t_0, stereo_init_ready_reject_count_);
}

bool Estimator::initialStructure()
{
    TicToc t_sfm;
    //check imu observibility
    {
        map<double, ImageFrame>::iterator frame_it;
        Vector3d sum_g;
        for (frame_it = all_image_frame.begin(), frame_it++; frame_it != all_image_frame.end(); frame_it++)
        {
            double dt = frame_it->second.pre_integration->sum_dt;
            Vector3d tmp_g = frame_it->second.pre_integration->delta_v / dt;
            sum_g += tmp_g;
        }
        Vector3d aver_g;
        aver_g = sum_g * 1.0 / ((int)all_image_frame.size() - 1);
        double var = 0;
        for (frame_it = all_image_frame.begin(), frame_it++; frame_it != all_image_frame.end(); frame_it++)
        {
            double dt = frame_it->second.pre_integration->sum_dt;
            Vector3d tmp_g = frame_it->second.pre_integration->delta_v / dt;
            var += (tmp_g - aver_g).transpose() * (tmp_g - aver_g);
            //cout << "frame g " << tmp_g.transpose() << endl;
        }
        var = sqrt(var / ((int)all_image_frame.size() - 1));
        //ROS_WARN("IMU variation %f!", var);
        if(var < 0.25)
        {
            ROS_INFO("IMU excitation not enouth!");
            //return false;
        }
    }
    // global sfm
    Quaterniond Q[frame_count + 1];
    Vector3d T[frame_count + 1];
    map<int, Vector3d> sfm_tracked_points;
    vector<SFMFeature> sfm_f;
    for (auto &it_per_id : f_manager.feature)
    {
        int imu_j = it_per_id.start_frame - 1;
        SFMFeature tmp_feature;
        tmp_feature.state = false;
        tmp_feature.id = it_per_id.feature_id;
        for (auto &it_per_frame : it_per_id.feature_per_frame)
        {
            imu_j++;
            Vector3d pts_j = it_per_frame.point;
            tmp_feature.observation.push_back(make_pair(imu_j, Eigen::Vector2d{pts_j.x(), pts_j.y()}));
        }
        sfm_f.push_back(tmp_feature);
    } 
    Matrix3d relative_R;
    Vector3d relative_T;
    int l;
    if (!relativePose(relative_R, relative_T, l))
    {
        ROS_INFO("Not enough features or parallax; Move device around");
        return false;
    }
    GlobalSFM sfm;
    if(!sfm.construct(frame_count + 1, Q, T, l,
              relative_R, relative_T,
              sfm_f, sfm_tracked_points))
    {
        ROS_DEBUG("global SFM failed!");
        marginalization_flag = MARGIN_OLD;
        return false;
    }

    //solve pnp for all frame
    map<double, ImageFrame>::iterator frame_it;
    map<int, Vector3d>::iterator it;
    frame_it = all_image_frame.begin( );
    for (int i = 0; frame_it != all_image_frame.end( ); frame_it++)
    {
        // provide initial guess
        cv::Mat r, rvec, t, D, tmp_r;
        if((frame_it->first) == Headers[i])
        {
            frame_it->second.is_key_frame = true;
            frame_it->second.R = Q[i].toRotationMatrix() * RIC[0].transpose();
            frame_it->second.T = T[i];
            i++;
            continue;
        }
        if((frame_it->first) > Headers[i])
        {
            i++;
        }
        Matrix3d R_inital = (Q[i].inverse()).toRotationMatrix();
        Vector3d P_inital = - R_inital * T[i];
        cv::eigen2cv(R_inital, tmp_r);
        cv::Rodrigues(tmp_r, rvec);
        cv::eigen2cv(P_inital, t);

        frame_it->second.is_key_frame = false;
        vector<cv::Point3f> pts_3_vector;
        vector<cv::Point2f> pts_2_vector;
        for (auto &id_pts : frame_it->second.points)
        {
            int feature_id = id_pts.first;
            for (auto &i_p : id_pts.second)
            {
                it = sfm_tracked_points.find(feature_id);
                if(it != sfm_tracked_points.end())
                {
                    Vector3d world_pts = it->second;
                    cv::Point3f pts_3(world_pts(0), world_pts(1), world_pts(2));
                    pts_3_vector.push_back(pts_3);
                    Vector2d img_pts = i_p.second.head<2>();
                    cv::Point2f pts_2(img_pts(0), img_pts(1));
                    pts_2_vector.push_back(pts_2);
                }
            }
        }
        cv::Mat K = (cv::Mat_<double>(3, 3) << 1, 0, 0, 0, 1, 0, 0, 0, 1);     
        if(pts_3_vector.size() < 6)
        {
            cout << "pts_3_vector size " << pts_3_vector.size() << endl;
            ROS_DEBUG("Not enough points for solve pnp !");
            return false;
        }
        if (! cv::solvePnP(pts_3_vector, pts_2_vector, K, D, rvec, t, 1))
        {
            ROS_DEBUG("solve pnp fail!");
            return false;
        }
        cv::Rodrigues(rvec, r);
        MatrixXd R_pnp,tmp_R_pnp;
        cv::cv2eigen(r, tmp_R_pnp);
        R_pnp = tmp_R_pnp.transpose();
        MatrixXd T_pnp;
        cv::cv2eigen(t, T_pnp);
        T_pnp = R_pnp * (-T_pnp);
        frame_it->second.R = R_pnp * RIC[0].transpose();
        frame_it->second.T = T_pnp;
    }
    if (visualInitialAlign())
        return true;
    else
    {
        ROS_INFO("misalign visual structure with IMU");
        return false;
    }

}

bool Estimator::visualInitialAlign()
{
    TicToc t_g;
    VectorXd x;
    //solve scale
    bool result = VisualIMUAlignment(all_image_frame, Bgs, g, x);
    if(!result)
    {
        ROS_DEBUG("solve g failed!");
        return false;
    }

    // change state
    for (int i = 0; i <= frame_count; i++)
    {
        Matrix3d Ri = all_image_frame[Headers[i]].R;
        Vector3d Pi = all_image_frame[Headers[i]].T;
        Ps[i] = Pi;
        Rs[i] = Ri;
        all_image_frame[Headers[i]].is_key_frame = true;
    }

    double s = (x.tail<1>())(0);
    for (int i = 0; i <= WINDOW_SIZE; i++)
    {
        pre_integrations[i]->repropagate(Vector3d::Zero(), Bgs[i]);
    }
    for (int i = frame_count; i >= 0; i--)
        Ps[i] = s * Ps[i] - Rs[i] * TIC[0] - (s * Ps[0] - Rs[0] * TIC[0]);
    int kv = -1;
    map<double, ImageFrame>::iterator frame_i;
    for (frame_i = all_image_frame.begin(); frame_i != all_image_frame.end(); frame_i++)
    {
        if(frame_i->second.is_key_frame)
        {
            kv++;
            Vs[kv] = frame_i->second.R * x.segment<3>(kv * 3);
        }
    }

    Matrix3d R0 = Utility::g2R(g);
    double yaw = Utility::R2ypr(R0 * Rs[0]).x();
    R0 = Utility::ypr2R(Eigen::Vector3d{-yaw, 0, 0}) * R0;
    g = R0 * g;
    //Matrix3d rot_diff = R0 * Rs[0].transpose();
    Matrix3d rot_diff = R0;
    for (int i = 0; i <= frame_count; i++)
    {
        Ps[i] = rot_diff * Ps[i];
        Rs[i] = rot_diff * Rs[i];
        Vs[i] = rot_diff * Vs[i];
    }
    ROS_DEBUG_STREAM("g0     " << g.transpose());
    ROS_DEBUG_STREAM("my R0  " << Utility::R2ypr(Rs[0]).transpose()); 

    f_manager.clearDepth();
    f_manager.triangulate(frame_count, Ps, Rs, tic, ric);

    return true;
}

bool Estimator::relativePose(Matrix3d &relative_R, Vector3d &relative_T, int &l)
{
    // find previous frame which contians enough correspondance and parallex with newest frame
    for (int i = 0; i < WINDOW_SIZE; i++)
    {
        vector<pair<Vector3d, Vector3d>> corres;
        corres = f_manager.getCorresponding(i, WINDOW_SIZE);
        if (corres.size() > 20)
        {
            double sum_parallax = 0;
            double average_parallax;
            for (int j = 0; j < int(corres.size()); j++)
            {
                Vector2d pts_0(corres[j].first(0), corres[j].first(1));
                Vector2d pts_1(corres[j].second(0), corres[j].second(1));
                double parallax = (pts_0 - pts_1).norm();
                sum_parallax = sum_parallax + parallax;

            }
            average_parallax = 1.0 * sum_parallax / int(corres.size());
            if(average_parallax * 460 > 30 && m_estimator.solveRelativeRT(corres, relative_R, relative_T))
            {
                l = i;
                ROS_DEBUG("average_parallax %f choose l %d and newest frame to triangulate the whole structure", average_parallax * 460, l);
                return true;
            }
        }
    }
    return false;
}

void Estimator::vector2double()
{
    for (int i = 0; i <= WINDOW_SIZE; i++)
    {
        para_Pose[i][0] = Ps[i].x();
        para_Pose[i][1] = Ps[i].y();
        para_Pose[i][2] = Ps[i].z();
        Quaterniond q{Rs[i]};
        para_Pose[i][3] = q.x();
        para_Pose[i][4] = q.y();
        para_Pose[i][5] = q.z();
        para_Pose[i][6] = q.w();

        if(USE_IMU)
        {
            para_SpeedBias[i][0] = Vs[i].x();
            para_SpeedBias[i][1] = Vs[i].y();
            para_SpeedBias[i][2] = Vs[i].z();

            para_SpeedBias[i][3] = Bas[i].x();
            para_SpeedBias[i][4] = Bas[i].y();
            para_SpeedBias[i][5] = Bas[i].z();

            para_SpeedBias[i][6] = Bgs[i].x();
            para_SpeedBias[i][7] = Bgs[i].y();
            para_SpeedBias[i][8] = Bgs[i].z();
        }
    }

    for (int i = 0; i < NUM_OF_CAM; i++)
    {
        para_Ex_Pose[i][0] = tic[i].x();
        para_Ex_Pose[i][1] = tic[i].y();
        para_Ex_Pose[i][2] = tic[i].z();
        Quaterniond q{ric[i]};
        para_Ex_Pose[i][3] = q.x();
        para_Ex_Pose[i][4] = q.y();
        para_Ex_Pose[i][5] = q.z();
        para_Ex_Pose[i][6] = q.w();
    }


    VectorXd dep = f_manager.getDepthVector();
    for (int i = 0; i < f_manager.getFeatureCount(); i++)
        para_Feature[i][0] = std::min(std::max(dep(i), kInvDepthMin), kInvDepthMax);

    para_Td[0][0] = td;
}

void Estimator::double2vector()
{
    Vector3d origin_R0 = Utility::R2ypr(Rs[0]);
    Vector3d origin_P0 = Ps[0];

    if (failure_occur)
    {
        origin_R0 = Utility::R2ypr(last_R0);
        origin_P0 = last_P0;
        failure_occur = 0;
    }

    if(USE_IMU)
    {
        Vector3d origin_R00 = Utility::R2ypr(Quaterniond(para_Pose[0][6],
                                                          para_Pose[0][3],
                                                          para_Pose[0][4],
                                                          para_Pose[0][5]).toRotationMatrix());
        double y_diff = origin_R0.x() - origin_R00.x();
        //TODO
        Matrix3d rot_diff = Utility::ypr2R(Vector3d(y_diff, 0, 0));
        if (abs(abs(origin_R0.y()) - 90) < 1.0 || abs(abs(origin_R00.y()) - 90) < 1.0)
        {
            ROS_DEBUG("euler singular point!");
            rot_diff = Rs[0] * Quaterniond(para_Pose[0][6],
                                           para_Pose[0][3],
                                           para_Pose[0][4],
                                           para_Pose[0][5]).toRotationMatrix().transpose();
        }

        for (int i = 0; i <= WINDOW_SIZE; i++)
        {

            Rs[i] = rot_diff * Quaterniond(para_Pose[i][6], para_Pose[i][3], para_Pose[i][4], para_Pose[i][5]).normalized().toRotationMatrix();
            
            Ps[i] = rot_diff * Vector3d(para_Pose[i][0] - para_Pose[0][0],
                                    para_Pose[i][1] - para_Pose[0][1],
                                    para_Pose[i][2] - para_Pose[0][2]) + origin_P0;


                Vs[i] = rot_diff * Vector3d(para_SpeedBias[i][0],
                                            para_SpeedBias[i][1],
                                            para_SpeedBias[i][2]);

                Bas[i] = Vector3d(para_SpeedBias[i][3],
                                  para_SpeedBias[i][4],
                                  para_SpeedBias[i][5]);

                Bgs[i] = Vector3d(para_SpeedBias[i][6],
                                  para_SpeedBias[i][7],
                                  para_SpeedBias[i][8]);
            
        }
    }
    else
    {
        for (int i = 0; i <= WINDOW_SIZE; i++)
        {
            Rs[i] = Quaterniond(para_Pose[i][6], para_Pose[i][3], para_Pose[i][4], para_Pose[i][5]).normalized().toRotationMatrix();
            
            Ps[i] = Vector3d(para_Pose[i][0], para_Pose[i][1], para_Pose[i][2]);
        }
    }

    if(USE_IMU)
    {
        for (int i = 0; i < NUM_OF_CAM; i++)
        {
            tic[i] = Vector3d(para_Ex_Pose[i][0],
                              para_Ex_Pose[i][1],
                              para_Ex_Pose[i][2]);
            ric[i] = Quaterniond(para_Ex_Pose[i][6],
                                 para_Ex_Pose[i][3],
                                 para_Ex_Pose[i][4],
                                 para_Ex_Pose[i][5]).normalized().toRotationMatrix();
        }
    }

    VectorXd dep = f_manager.getDepthVector();
    for (int i = 0; i < f_manager.getFeatureCount(); i++)
        dep(i) = para_Feature[i][0];
    f_manager.setDepth(dep);

    if(USE_IMU)
        td = para_Td[0][0];

    applyLowFlowStationaryLock();
}

bool Estimator::failureDetection()
{
    if (f_manager.last_track_num < 2)
    {
        ++little_feature_count_;
        const int grace_frames = std::max(8, FAILURE_GRACE_FRAMES);
        const double header = Headers[WINDOW_SIZE] > 0.0 ? Headers[WINDOW_SIZE] : latest_time;
        if (BLIND_ENABLE && USE_IMU && solver_flag == NON_LINEAR && !blind_active)
        {
            visual_state = VISUAL_BLIND;
            pending_state_ = VISUAL_BLIND;
            pending_count_ = 0;
            enterBlind(header);
        }
        if (little_feature_count_ < grace_frames)
        {
            ROS_WARN("VINS little-feature blind grace %d/%d: backend_tracks=%d frontend_pts=%d",
                     little_feature_count_, grace_frames, f_manager.last_track_num,
                     latest_frontend_quality_.total_points);
            return false;
        }
        ROS_INFO(" little feature %d for %d frames", f_manager.last_track_num, little_feature_count_);
        return true;
    }
    little_feature_count_ = 0;

    if (!Ps[WINDOW_SIZE].allFinite() || !Rs[WINDOW_SIZE].allFinite())
    {
        ROS_WARN("non-finite pose state");
        return true;
    }
    if (USE_IMU && (!Vs[WINDOW_SIZE].allFinite() ||
                    !Bas[WINDOW_SIZE].allFinite() ||
                    !Bgs[WINDOW_SIZE].allFinite()))
    {
        ROS_WARN("non-finite speed/bias state");
        return true;
    }

    const double acc_bias_max = std::max(0.1, BLIND_BIAS_ACC_MAX);
    const double gyr_bias_max = std::max(0.01, BLIND_BIAS_GYR_MAX);
    const double immediate_frame_jump =
        last_state_valid_ ? (Ps[WINDOW_SIZE] - last_P).norm() : 0.0;
    if (last_state_valid_ && immediate_frame_jump > FAILURE_POSE_JUMP_HARD)
    {
        ROS_WARN("VINS failure: hard pose jump %.2fm (instant)", immediate_frame_jump);
        return true;
    }
    const bool bias_bad = USE_IMU &&
                          (Bas[WINDOW_SIZE].norm() > acc_bias_max ||
                           Bgs[WINDOW_SIZE].norm() > gyr_bias_max);
    if (bias_bad)
    {
        ++bias_failure_count_;

        const double now = Headers[WINDOW_SIZE] > 0.0 ? Headers[WINDOW_SIZE] : latest_time;
        const double nonlinear_age =
            (nonlinear_start_time_ > 0.0 && now > nonlinear_start_time_) ?
            (now - nonlinear_start_time_) : 0.0;
        const bool startup_grace =
            nonlinear_frame_count_ < 45 || nonlinear_age < 4.0;
        const bool bias_grace =
            startup_grace || bias_failure_count_ < std::max(3, FAILURE_GRACE_FRAMES);

        if (bias_grace)
        {
            bool clamped = false;
            for (int i = 0; i <= frame_count; i++)
            {
                const double ba_norm = Bas[i].norm();
                if (ba_norm > acc_bias_max)
                {
                    Bas[i] *= acc_bias_max / ba_norm;
                    clamped = true;
                }
                const double bg_norm = Bgs[i].norm();
                if (bg_norm > gyr_bias_max)
                {
                    Bgs[i] *= gyr_bias_max / bg_norm;
                    clamped = true;
                }

                para_SpeedBias[i][3] = Bas[i].x();
                para_SpeedBias[i][4] = Bas[i].y();
                para_SpeedBias[i][5] = Bas[i].z();
                para_SpeedBias[i][6] = Bgs[i].x();
                para_SpeedBias[i][7] = Bgs[i].y();
                para_SpeedBias[i][8] = Bgs[i].z();
            }
            if (clamped)
            {
                for (int i = 0; i <= frame_count; i++)
                {
                    if (pre_integrations[i] != nullptr)
                        pre_integrations[i]->repropagate(Bas[i], Bgs[i]);
                }
                if (tmp_pre_integration != nullptr)
                    tmp_pre_integration->repropagate(Bas[frame_count], Bgs[frame_count]);
            }
            ROS_WARN("VINS bias transient clamped %d/%d: Ba=%.3f Bg=%.3f age=%.2fs frames=%d",
                     bias_failure_count_, std::max(3, FAILURE_GRACE_FRAMES),
                     Bas[WINDOW_SIZE].norm(), Bgs[WINDOW_SIZE].norm(),
                     nonlinear_age, nonlinear_frame_count_);
            return false;
        }

        if (Bas[WINDOW_SIZE].norm() > acc_bias_max)
            ROS_INFO(" big IMU acc bias estimation %f", Bas[WINDOW_SIZE].norm());
        if (Bgs[WINDOW_SIZE].norm() > gyr_bias_max)
            ROS_INFO(" big IMU gyr bias estimation %f", Bgs[WINDOW_SIZE].norm());
        return true;
    }
    bias_failure_count_ = 0;

    if (!last_state_valid_)
        return false;

    Vector3d tmp_P = Ps[WINDOW_SIZE];
    const double frame_jump = (tmp_P - last_P).norm();

    // Hard, unrecoverable divergence -> kill immediately, no grace.
    if (frame_jump > FAILURE_POSE_JUMP_HARD)
    {
        ROS_WARN("VINS failure: hard pose jump %.2fm (instant)", frame_jump);
        return true;
    }

    Matrix3d tmp_R = Rs[WINDOW_SIZE];
    Matrix3d delta_R = tmp_R.transpose() * last_R;
    Quaterniond delta_Q(delta_R);
    double delta_angle =
        acos(std::max(-1.0, std::min(1.0, delta_Q.w()))) * 2.0 / 3.14 * 180.0;

    const bool visually_untrusted =
        isBlind() || isVisualDegraded() ||
        latest_feature_quality_metrics_[3] > 0.50 ||
        latest_feature_quality_metrics_[2] < 0.05;

    // Soft anomalies: a single high-dynamic transient (e.g. a momentary quality
    // collapse during fast re-detection) must NOT kill the system. Require the
    // anomaly to persist for FAILURE_GRACE_FRAMES consecutive frames.
    const bool soft_anomaly =
        (USE_IMU && Vs[WINDOW_SIZE].norm() > 6.0) ||
        (frame_jump > 1.0) ||
        (visually_untrusted && frame_jump > 0.35) ||
        (fabs(tmp_P.z() - last_P.z()) > 0.60) ||
        (delta_angle > 50);

    if (soft_anomaly)
        ++soft_failure_count_;
    else
        soft_failure_count_ = 0;

    if (soft_failure_count_ >= std::max(1, FAILURE_GRACE_FRAMES))
    {
        ROS_WARN("VINS failure: %d consecutive anomalies, jump %.2fm vel %.2f angle %.1f health=%d new=%.2f hq=%.2f",
                 soft_failure_count_, frame_jump, Vs[WINDOW_SIZE].norm(), delta_angle,
                 getHealthCode(), latest_feature_quality_metrics_[3],
                 latest_feature_quality_metrics_[2]);
        return true;
    }
    if (soft_anomaly)
        ROS_WARN_THROTTLE(0.5, "VINS soft anomaly %d/%d: jump %.2fm health=%d (grace)",
                          soft_failure_count_, FAILURE_GRACE_FRAMES, frame_jump, getHealthCode());
    return false;
}

std::vector<char> Estimator::selectGoodFeatures() const
{
    std::vector<char> selected;
    if (!GOOD_FEATURE_ENABLE)
        return selected;

    int candidate_count = 0;
    for (const auto &it_per_id : f_manager.feature)
    {
        if (!featureReadyForOptimization(it_per_id))
            continue;
        candidate_count++;
    }
    if (candidate_count <= 0)
        return selected;

    selected.assign(candidate_count, 0);
    const int budget = std::min(candidate_count, std::max(1, GOOD_FEATURE_BUDGET));

    struct Candidate
    {
        int index;
        Eigen::Matrix3d info;
        double score;
        int track_len;
        double parallax_px;
    };
    std::vector<Candidate> candidates;
    candidates.reserve(candidate_count);

    int feature_index = -1;
    for (const auto &it_per_id : f_manager.feature)
    {
        if (!featureReadyForOptimization(it_per_id))
            continue;
        ++feature_index;

        const auto &anchor = it_per_id.feature_per_frame.front();
        Eigen::Vector3d b = anchor.point.normalized();
        if (!b.allFinite() || b.norm() < 1e-6)
            b = Eigen::Vector3d::UnitZ();
        const double score = computeFeatureQualityScore(it_per_id);
        const int track_len = std::max(1, static_cast<int>(it_per_id.feature_per_frame.size()));
        const double parallax_px = estimateFeatureParallaxPx(it_per_id);
        const double reproj_px = estimateFeatureReprojectionErrorPx(it_per_id);
        const double stereo_consistency = estimateStereoDepthConsistency(it_per_id);

        const bool weak_new_track =
            track_len < std::max(1, GOOD_FEATURE_NEW_MIN_TRACK_LENGTH) &&
            score < GOOD_FEATURE_NEW_MIN_QUALITY;
        const bool low_quality = score < GOOD_FEATURE_MIN_QUALITY;
        const bool low_parallax =
            track_len >= GOOD_FEATURE_MIN_TRACK_LENGTH &&
            parallax_px < GOOD_FEATURE_MIN_PARALLAX;
        const bool bad_reproj =
            std::isfinite(reproj_px) && reproj_px > GOOD_FEATURE_MAX_REPROJ_ERROR;

        if (weak_new_track || low_quality || low_parallax || bad_reproj)
            continue;

        // Projection residuals constrain camera translation on the tangent
        // plane perpendicular to the bearing. Using b*b^T overstates the
        // forward/radial direction and can drop rare off-axis anchors.
        const Eigen::Matrix3d tangent_info = Eigen::Matrix3d::Identity() - b * b.transpose();
        double depth_weight = 1.0;
        if (std::isfinite(it_per_id.estimated_depth) && it_per_id.estimated_depth > 0.0)
            depth_weight = std::max(0.25, std::min(2.0, 4.0 / it_per_id.estimated_depth));
        const double stereo_weight = std::isfinite(stereo_consistency)
                                         ? std::max(0.70, std::min(1.0, stereo_consistency))
                                         : 0.95;
        const double final_score = std::max(GOOD_FEATURE_MIN_SCALE, score) * stereo_weight;
        const Eigen::Matrix3d info =
            final_score * depth_weight * depth_weight * tangent_info;

        candidates.push_back(Candidate{feature_index, info, final_score,
                                        track_len, parallax_px});
    }

    if (candidates.empty())
        return selected;

    if (budget >= static_cast<int>(candidates.size()))
    {
        for (const auto &candidate : candidates)
            selected[candidate.index] = 1;
        return selected;
    }

    std::sort(candidates.begin(), candidates.end(),
              [](const Candidate &a, const Candidate &b)
              {
                  if (fabs(a.score - b.score) > 1e-6)
                      return a.score > b.score;
                  if (a.track_len != b.track_len)
                      return a.track_len > b.track_len;
                  return a.parallax_px > b.parallax_px;
              });

    Eigen::Matrix3d info = 1e-6 * Eigen::Matrix3d::Identity();
    const int target_budget = std::min(budget, static_cast<int>(candidates.size()));
    for (int pick = 0; pick < target_budget; pick++)
    {
        double best_gain = -1e18;
        int best_idx = -1;
        Eigen::Matrix3d best_info = info;
        for (size_t i = 0; i < candidates.size(); i++)
        {
            const Candidate &c = candidates[i];
            if (selected[c.index])
                continue;
            if (c.track_len < GOOD_FEATURE_MIN_TRACK_LENGTH)
                continue;

            Eigen::Matrix3d trial = info + c.info;
            const double gain = safeLogDet(trial) - safeLogDet(info);
            if (gain > best_gain)
            {
                best_gain = gain;
                best_idx = static_cast<int>(i);
                best_info = trial;
            }
        }

        if (best_idx < 0)
            break;

        selected[candidates[best_idx].index] = 1;
        info = best_info;
    }

    return selected;
}

double Estimator::estimateFeatureParallaxPx(const FeaturePerId &feature) const
{
    if (feature.feature_per_frame.size() < 2)
        return 0.0;

    const Eigen::Vector2d anchor_uv = feature.feature_per_frame.front().point.head<2>();
    double best = 0.0;
    for (size_t i = 1; i < feature.feature_per_frame.size(); i++)
    {
        const Eigen::Vector2d uv_i = feature.feature_per_frame[i].point.head<2>();
        best = std::max(best, (uv_i - anchor_uv).norm() * FOCAL_LENGTH);
    }
    return best;
}

double Estimator::estimateFeatureReprojectionErrorPx(const FeaturePerId &feature) const
{
    if (!std::isfinite(feature.estimated_depth) || feature.estimated_depth <= 0.0)
        return std::numeric_limits<double>::infinity();

    int imu_i = feature.start_frame;
    int imu_j = imu_i - 1;
    const Vector3d pts_i = feature.feature_per_frame.front().point;
    const double depth = feature.estimated_depth;
    double err = 0.0;
    int err_cnt = 0;

    for (const auto &frame_obs : feature.feature_per_frame)
    {
        imu_j++;
        if (imu_i != imu_j)
        {
            const double tmp_error = reprojectionError(Rs[imu_i], Ps[imu_i], ric[0], tic[0],
                                                       Rs[imu_j], Ps[imu_j], ric[0], tic[0],
                                                       depth, pts_i, frame_obs.point);
            if (!std::isfinite(tmp_error))
                return std::numeric_limits<double>::infinity();
            err += tmp_error * FOCAL_LENGTH;
            err_cnt++;
        }

        if (STEREO && frame_obs.is_stereo)
        {
            const double tmp_error = reprojectionError(Rs[imu_i], Ps[imu_i], ric[0], tic[0],
                                                       Rs[imu_j], Ps[imu_j], ric[1], tic[1],
                                                       depth, pts_i, frame_obs.pointRight);
            if (!std::isfinite(tmp_error))
                return std::numeric_limits<double>::infinity();
            err += tmp_error * FOCAL_LENGTH;
            err_cnt++;
        }
    }

    if (err_cnt <= 0)
        return std::numeric_limits<double>::infinity();
    return err / static_cast<double>(err_cnt);
}

double Estimator::estimateStereoDepthConsistency(const FeaturePerId &feature) const
{
    if (!STEREO || !std::isfinite(feature.estimated_depth) || feature.estimated_depth <= 0.1)
        return std::numeric_limits<double>::quiet_NaN();

    std::vector<double> rel_errors;
    int imu_j = feature.start_frame - 1;
    for (const auto &frame_obs : feature.feature_per_frame)
    {
        imu_j++;
        if (!frame_obs.is_stereo || imu_j < 0 || imu_j > WINDOW_SIZE)
            continue;

        Eigen::Matrix<double, 3, 4> leftPose;
        Eigen::Vector3d t0 = Ps[imu_j] + Rs[imu_j] * tic[0];
        Eigen::Matrix3d R0 = Rs[imu_j] * ric[0];
        leftPose.leftCols<3>() = R0.transpose();
        leftPose.rightCols<1>() = -R0.transpose() * t0;

        Eigen::Matrix<double, 3, 4> rightPose;
        Eigen::Vector3d t1 = Ps[imu_j] + Rs[imu_j] * tic[1];
        Eigen::Matrix3d R1 = Rs[imu_j] * ric[1];
        rightPose.leftCols<3>() = R1.transpose();
        rightPose.rightCols<1>() = -R1.transpose() * t1;

        Eigen::Vector2d point0 = frame_obs.point.head<2>();
        Eigen::Vector2d point1 = frame_obs.pointRight.head<2>();
        Eigen::Matrix4d design_matrix = Eigen::Matrix4d::Zero();
        design_matrix.row(0) = point0[0] * leftPose.row(2) - leftPose.row(0);
        design_matrix.row(1) = point0[1] * leftPose.row(2) - leftPose.row(1);
        design_matrix.row(2) = point1[0] * rightPose.row(2) - rightPose.row(0);
        design_matrix.row(3) = point1[1] * rightPose.row(2) - rightPose.row(1);
        Eigen::Vector4d triangulated_point =
            Eigen::JacobiSVD<Eigen::Matrix4d>(design_matrix, Eigen::ComputeFullV).matrixV().rightCols<1>();
        if (std::fabs(triangulated_point(3)) < 1e-9)
            continue;
        Eigen::Vector3d point3d = triangulated_point.head<3>() / triangulated_point(3);

        Eigen::Vector3d localPoint = leftPose.leftCols<3>() * point3d + leftPose.rightCols<1>();
        const double stereo_depth = localPoint.z();
        if (!std::isfinite(stereo_depth) || stereo_depth <= 0.1 || stereo_depth > 100.0)
            continue;

        const double rel =
            std::fabs(stereo_depth - feature.estimated_depth) /
            std::max(0.5, std::max(stereo_depth, feature.estimated_depth));
        if (std::isfinite(rel))
            rel_errors.push_back(rel);
    }

    if (rel_errors.empty())
        return std::numeric_limits<double>::quiet_NaN();

    const double rel_med = safeMedian(rel_errors);
    return clampUnit(1.0 / (1.0 + 4.0 * rel_med));
}

double Estimator::computeFeatureQualityScore(const FeaturePerId &feature) const
{
    if (feature.feature_per_frame.empty())
        return 0.0;

    double frontend_quality = 0.0;
    int quality_cnt = 0;
    for (const auto &obs : feature.feature_per_frame)
    {
        frontend_quality += std::max(0.05, std::min(1.0, obs.quality));
        quality_cnt++;
        if (obs.is_stereo)
        {
            frontend_quality += std::max(0.05, std::min(1.0, obs.qualityRight));
            quality_cnt++;
        }
    }
    frontend_quality = quality_cnt > 0 ? frontend_quality / static_cast<double>(quality_cnt) : 0.0;

    const int track_len = std::max(1, static_cast<int>(feature.feature_per_frame.size()));
    const double len_weight = clampUnit((track_len + 2.0) / 8.0);

    const double parallax_px = estimateFeatureParallaxPx(feature);
    double parallax_weight = 0.55;
    if (track_len >= 2)
        parallax_weight = clampUnit(parallax_px / std::max(1.0, 2.0 * GOOD_FEATURE_MIN_PARALLAX));

    const double reproj_px = estimateFeatureReprojectionErrorPx(feature);
    double reproj_weight = 0.65;
    if (std::isfinite(reproj_px))
    {
        reproj_weight = 1.0 / (1.0 + reproj_px / std::max(0.5, GOOD_FEATURE_MAX_REPROJ_ERROR));
    }
    else if (track_len >= std::max(3, GOOD_FEATURE_MIN_TRACK_LENGTH))
    {
        reproj_weight = 0.45;
    }

    const double score =
        0.45 * frontend_quality +
        0.20 * len_weight +
        0.20 * parallax_weight +
        0.15 * reproj_weight;
    return clampUnit(score);
}

std::array<double, 4> Estimator::computeFeatureQualityMetrics() const
{
    std::vector<double> mature_scores;
    std::vector<double> fallback_scores;
    int bad_cnt = 0;
    int long_hq_cnt = 0;
    int new_cnt = 0;
    int active_cnt = 0;
    int mature_cnt = 0;

    const int mature_track_len = std::max(3, GOOD_FEATURE_MIN_TRACK_LENGTH);

    for (const auto &feature : f_manager.feature)
    {
        if (feature.endFrame() != frame_count)
            continue;

        const int track_len = std::max(1, static_cast<int>(feature.feature_per_frame.size()));
        active_cnt++;
        if (track_len <= 2)
            new_cnt++;

        double frontend_quality = 0.0;
        int quality_cnt = 0;
        for (const auto &obs : feature.feature_per_frame)
        {
            frontend_quality += std::max(0.05, std::min(1.0, obs.quality));
            quality_cnt++;
            if (obs.is_stereo)
            {
                frontend_quality += std::max(0.05, std::min(1.0, obs.qualityRight));
                quality_cnt++;
            }
        }
        frontend_quality = quality_cnt > 0 ? frontend_quality / static_cast<double>(quality_cnt) : 0.0;

        const double parallax_px = estimateFeatureParallaxPx(feature);
        const bool mature_track =
            track_len >= mature_track_len &&
            (std::isfinite(feature.estimated_depth) && feature.estimated_depth > 0.0 ||
             parallax_px >= 0.5 * GOOD_FEATURE_MIN_PARALLAX);

        if (mature_track)
        {
            const double score = computeFeatureQualityScore(feature);
            mature_scores.push_back(score);
            mature_cnt++;

            if (score < 0.40)
                bad_cnt++;
            if (track_len >= 10 && score >= 0.75)
                long_hq_cnt++;
        }
        else if (track_len >= 2)
        {
            fallback_scores.push_back(std::max(0.20, frontend_quality));
        }
    }

    if (active_cnt <= 0)
        return {0.0, 1.0, 0.0, 1.0};

    double qmed = 0.0;
    double bad_ratio = 1.0;
    double hq_long_ratio = 0.0;
    if (!mature_scores.empty())
    {
        qmed = safeMedian(mature_scores);
        bad_ratio = static_cast<double>(bad_cnt) / static_cast<double>(mature_cnt);
        hq_long_ratio = static_cast<double>(long_hq_cnt) / static_cast<double>(mature_cnt);
    }
    else if (!fallback_scores.empty())
    {
        qmed = safeMedian(fallback_scores);
        int fallback_bad = 0;
        for (double score : fallback_scores)
            fallback_bad += score < 0.35 ? 1 : 0;
        bad_ratio = static_cast<double>(fallback_bad) / static_cast<double>(fallback_scores.size());
    }
    else
    {
        qmed = clampUnit(0.5 * latest_frontend_quality_.lk_keep_ratio +
                         0.5 * latest_frontend_quality_.coverage_ratio);
        bad_ratio = qmed < 0.35 ? 1.0 : 0.0;
    }

    return {
        qmed,
        bad_ratio,
        hq_long_ratio,
        static_cast<double>(new_cnt) / static_cast<double>(active_cnt)};
}

bool Estimator::isBlind() const
{
    return BLIND_ENABLE && visual_state == VISUAL_BLIND && blind_active;
}

bool Estimator::isVisualDegraded() const
{
    return BLIND_ENABLE && visual_state == VISUAL_DEGRADED;
}

bool Estimator::isLowFlowStationary() const
{
    return LOW_FLOW_ZUPT_ENABLE && solver_flag == NON_LINEAR &&
           latest_frontend_quality_.tracked_after_lk >= LOW_FLOW_ZUPT_MIN_TRACKS &&
           latest_frontend_quality_.lk_keep_ratio > 0.80 &&
           latest_frontend_quality_.mean_pixel_flow >= 0.0 &&
           latest_frontend_quality_.mean_pixel_flow < LOW_FLOW_ZUPT_FLOW;
}

void Estimator::applyLowFlowStationaryLock()
{
    if (!USE_IMU)
        return;

    if (solver_flag == NON_LINEAR && !home_loop_origin_valid_ && Ps[frame_count].allFinite())
    {
        home_loop_origin_ = Ps[frame_count];
        home_loop_origin_valid_ = true;
        home_loop_max_radius_ = 0.0;
        ROS_WARN("VINS home-loop origin set: P=(%.3f %.3f %.3f)",
                 home_loop_origin_.x(), home_loop_origin_.y(), home_loop_origin_.z());
    }

    if (home_loop_origin_valid_ && Ps[frame_count].allFinite())
        home_loop_max_radius_ = std::max(home_loop_max_radius_,
                                         (Ps[frame_count] - home_loop_origin_).norm());

    if (!isLowFlowStationary())
    {
        if (low_flow_lock_active_)
            ROS_WARN_THROTTLE(1.0, "VINS low-flow stationary lock released: flow=%.3f tracks=%d",
                              latest_frontend_quality_.mean_pixel_flow,
                              latest_frontend_quality_.tracked_after_lk);
        low_flow_lock_active_ = false;
        home_loop_active_ = false;
        home_loop_static_count_ = 0;
        return;
    }

    if (!low_flow_lock_active_)
    {
        low_flow_lock_P_ = Ps[frame_count].allFinite() ? Ps[frame_count] : latest_P;
        low_flow_lock_active_ = low_flow_lock_P_.allFinite();
        ROS_WARN_THROTTLE(1.0, "VINS low-flow stationary lock anchor: flow=%.3f tracks=%d P=(%.3f %.3f %.3f)",
                          latest_frontend_quality_.mean_pixel_flow,
                          latest_frontend_quality_.tracked_after_lk,
                          low_flow_lock_P_.x(), low_flow_lock_P_.y(), low_flow_lock_P_.z());
    }
    if (!low_flow_lock_active_)
        return;

    home_loop_static_count_++;
    if (HOME_LOOP_ENABLE && home_loop_origin_valid_ &&
        home_loop_static_count_ >= std::max(1, HOME_LOOP_MIN_STATIC_FRAMES) &&
        home_loop_max_radius_ >= std::max(0.0, HOME_LOOP_MIN_TRAVEL))
    {
        const Eigen::Vector3d drift = low_flow_lock_P_ - home_loop_origin_;
        const double drift_norm = drift.norm();
        const double capture_radius = std::max(0.01, HOME_LOOP_CAPTURE_RADIUS);
        if (std::isfinite(drift_norm) && drift_norm < capture_radius)
        {
            const double gain = clampUnit(HOME_LOOP_GAIN);
            if (gain > 0.0 && drift_norm > 1e-4)
            {
                low_flow_lock_P_ -= gain * drift;
                home_loop_active_ = true;
                ROS_WARN_THROTTLE(1.0,
                                  "VINS home-loop closure: drift %.3f -> %.3f, max_radius=%.3f, gain=%.2f",
                                  drift_norm,
                                  (low_flow_lock_P_ - home_loop_origin_).norm(),
                                  home_loop_max_radius_,
                                  gain);
            }
        }
        else if (std::isfinite(drift_norm))
        {
            ROS_WARN_THROTTLE(2.0,
                              "VINS home-loop not applied: drift %.3f exceeds capture radius %.3f",
                              drift_norm,
                              capture_radius);
        }
    }

    const int window = std::max(1, LOW_FLOW_ZUPT_WINDOW);
    const int start = std::max(0, frame_count - window + 1);
    const double speed_before = Vs[frame_count].norm();
    for (int i = start; i <= frame_count; ++i)
    {
        Ps[i] = low_flow_lock_P_;
        para_Pose[i][0] = low_flow_lock_P_.x();
        para_Pose[i][1] = low_flow_lock_P_.y();
        para_Pose[i][2] = low_flow_lock_P_.z();
        Vs[i].setZero();
        para_SpeedBias[i][0] = 0.0;
        para_SpeedBias[i][1] = 0.0;
        para_SpeedBias[i][2] = 0.0;
    }
    latest_P = low_flow_lock_P_;
    latest_V.setZero();

    ROS_WARN_THROTTLE(1.0, "VINS low-flow stationary lock: flow=%.3f tracks=%d keep=%.2f vel %.2f -> 0.00 P=(%.3f %.3f %.3f)",
                      latest_frontend_quality_.mean_pixel_flow,
                      latest_frontend_quality_.tracked_after_lk,
                      latest_frontend_quality_.lk_keep_ratio,
                      speed_before,
                      low_flow_lock_P_.x(), low_flow_lock_P_.y(), low_flow_lock_P_.z());
}

bool Estimator::allowImuPropagateOutput() const
{
    if (solver_flag != NON_LINEAR)
        return false;
    if (isLowFlowStationary())
        return true;
    return !isBlind() && !isVisualDegraded();
}

int Estimator::getFrontendPointCount() const
{
    return latest_frontend_quality_.total_points;
}

double Estimator::getFrontendCoverageRatio() const
{
    return latest_frontend_quality_.coverage_ratio;
}

double Estimator::getFrontendKeepRatio() const
{
    return latest_frontend_quality_.lk_keep_ratio;
}

double Estimator::getLatestEstimatorLatency() const
{
    return latest_estimator_latency_ms_;
}

double Estimator::getFeatureQualityMedian() const
{
    return latest_feature_quality_metrics_[0];
}

double Estimator::getFeatureQualityBadRatio() const
{
    return latest_feature_quality_metrics_[1];
}

double Estimator::getFeatureHighQualityLongRatio() const
{
    return latest_feature_quality_metrics_[2];
}

double Estimator::getFeatureNewRatio() const
{
    return latest_feature_quality_metrics_[3];
}

int Estimator::getHealthCode() const
{
    if (solver_flag != NON_LINEAR)
        return 3;
    if (isBlind())
        return 2;
    if (isVisualDegraded())
        return 1;
    return 0;
}

double Estimator::blindDuration(double header) const
{
    if (blind_start_time < 0.0)
        return 0.0;
    return std::max(0.0, header - blind_start_time);
}

void Estimator::enterBlind(double header)
{
    if (blind_active)
        return;

    blind_active = true;
    blind_anchor_valid = USE_IMU && solver_flag == NON_LINEAR;
    blind_start_time = header;
    blind_ba0 = Bas[frame_count];
    blind_bg0 = Bgs[frame_count];
    blind_acc_body0 = acc_0;
    blind_v0 = Vs[frame_count];
    if (blind_acc_body0.norm() > 1e-3)
        blind_acc_body0.normalize();
    ROS_WARN("VINS visual BLIND enter: tracks=%d parallax=%.2f", visual_track_num, visual_parallax);
}

void Estimator::exitBlind(double header)
{
    if (!blind_active)
        return;

    ROS_WARN("VINS visual BLIND exit after %.2fs: tracks=%d parallax=%.2f",
             blindDuration(header), visual_track_num, visual_parallax);
    blind_active = false;
}

bool Estimator::shouldSkipVisualFrame(const map<int, vector<pair<int, FeatureObservation>>> &image,
                                      const FrontendQuality &frontend_quality) const
{
    const VisualHealthSnapshot snapshot =
        makeVisualHealthSnapshot(image, frontend_quality, visual_track_num, visual_parallax);
    const VisualHealthMonitor monitor;
    return monitor.shouldSkipVisualFrame(snapshot,
                                         BLIND_ENABLE,
                                         USE_IMU,
                                         solver_flag == NON_LINEAR,
                                         visual_state,
                                         blind_active);
}

void Estimator::updateVisualHealth(const map<int, vector<pair<int, FeatureObservation>>> &image,
                                   const FrontendQuality &frontend_quality,
                                   double header)
{
    if (!BLIND_ENABLE || solver_flag != NON_LINEAR)
        return;

    visual_track_num = f_manager.last_track_num;
    visual_parallax = f_manager.last_average_parallax;
    latest_feature_quality_metrics_ = computeFeatureQualityMetrics();
    VisualHealthSnapshot snapshot =
        makeVisualHealthSnapshot(image, frontend_quality, visual_track_num, visual_parallax);
    snapshot.quality_median = latest_feature_quality_metrics_[0];
    snapshot.quality_bad_ratio = latest_feature_quality_metrics_[1];
    snapshot.high_quality_long_ratio = latest_feature_quality_metrics_[2];
    snapshot.new_feature_ratio = latest_feature_quality_metrics_[3];
    ROS_INFO_THROTTLE(1.0,
                      "VINS photometric health: photo=%.2f mean=%.1f dark=%.3f sat=%.3f contrast=%.1f blur=%.1f",
                      snapshot.photometric_health,
                      snapshot.brightness_mean,
                      snapshot.dark_ratio,
                      snapshot.saturated_ratio,
                      snapshot.contrast_std,
                      snapshot.blur_score);
    const VisualHealthMonitor monitor;
    VisualState target = monitor.classify(snapshot,
                                          BLIND_ENABLE,
                                          solver_flag == NON_LINEAR,
                                          visual_state,
                                          blind_active);
    // A frame is "blind" only when visual INFORMATION is genuinely missing.
    // qmed/bad/hq/new all measure TRACK AGE, not visual viability: during fast
    // re-detection the window fills with track_cnt==1 features, so those stats
    // collapse even though the camera sees plenty of well-distributed, RANSAC-
    // consistent structure. The blind decision therefore keys ONLY on
    // track-age-independent geometry: detectable points, coverage, RANSAC
    // survivors. High-flow re-detection is explicitly protected.
    const int detectable_pts =
        snapshot.total_points > 0 ? snapshot.total_points : snapshot.image_points;
    const int ransac_survivors = snapshot.tracked_after_ransac;
    const bool fast_redetect =
        snapshot.mean_pixel_flow > BLIND_FLOW_DYN &&
        snapshot.coverage_ratio >= 0.6 &&
        detectable_pts >= BLIND_PTS_OK &&
        ransac_survivors >= BLIND_PTS_OK / 2;
    const bool ransac_collapsed =
        (snapshot.prev_points >= BLIND_PTS_OK) && (ransac_survivors < BLIND_PTS_BLIND);
    const bool hard_reset_frame =
        !fast_redetect &&
        (detectable_pts < BLIND_PTS_MIN ||
         snapshot.coverage_ratio < 0.35 ||
         ransac_collapsed);
    if (hard_reset_frame)
        target = VISUAL_BLIND;

    if (target == visual_state)
    {
        pending_count_ = 0;
    }
    else
    {
        if (target == pending_state_)
            ++pending_count_;
        else
        {
            pending_state_ = target;
            pending_count_ = 1;
        }

        int need = 1;
        if (hard_reset_frame)
            need = 1;
        else if (target == VISUAL_BLIND)
            need = std::max(1, BLIND_DEBOUNCE_FRAMES);
        else if (target == VISUAL_DEGRADED)
            need = std::max(1, QUALITY_DEGRADED_DEBOUNCE);
        else if (visual_state != VISUAL_HEALTHY)
            need = std::max(3, BLIND_DEBOUNCE_FRAMES);
        if (pending_count_ >= need)
        {
            const VisualState old = visual_state;
            visual_state = target;
            pending_count_ = 0;

            if (visual_state == VISUAL_BLIND && old != VISUAL_BLIND)
                enterBlind(header);
            else if (old == VISUAL_BLIND && visual_state != VISUAL_BLIND)
                exitBlind(header);

            ROS_WARN("VINS visual state %d -> %d, pts=%d, tracks=%d, flow=%.1f, eig=%.3g, par=%.2f, cov=%.2f, qmed=%.2f, bad=%.2f, hq=%.2f, new=%.2f, photo=%.2f, mean=%.1f, dark=%.3f, sat=%.3f, blur=%.1f",
                     static_cast<int>(old), static_cast<int>(visual_state),
                     frontend_quality.total_points, visual_track_num,
                     frontend_quality.mean_pixel_flow,
                     frontend_quality.mean_track_eigen, visual_parallax,
                     frontend_quality.coverage_ratio,
                     snapshot.quality_median,
                     snapshot.quality_bad_ratio,
                     snapshot.high_quality_long_ratio,
                     snapshot.new_feature_ratio,
                     snapshot.photometric_health,
                     snapshot.brightness_mean,
                     snapshot.dark_ratio,
                     snapshot.saturated_ratio,
                     snapshot.blur_score);
        }
    }

    if (visual_state == VISUAL_HEALTHY && blind_anchor_valid)
    {
        blind_anchor_valid = false;
        blind_start_time = -1.0;
    }
}

bool Estimator::isLowDynamic(int frame_index) const
{
    if (frame_index < 0 || frame_index > WINDOW_SIZE)
        return false;
    const vector<Vector3d> &acc_buf = linear_acceleration_buf[frame_index];
    const vector<Vector3d> &gyr_buf = angular_velocity_buf[frame_index];
    if (acc_buf.size() < 5 || gyr_buf.size() != acc_buf.size())
        return false;

    Vector3d mean_acc = Vector3d::Zero();
    double mean_gyr_norm = 0.0;
    for (size_t i = 0; i < acc_buf.size(); i++)
    {
        mean_acc += acc_buf[i];
        mean_gyr_norm += gyr_buf[i].norm();
    }
    mean_acc /= static_cast<double>(acc_buf.size());
    mean_gyr_norm /= static_cast<double>(gyr_buf.size());

    double acc_var = 0.0;
    for (size_t i = 0; i < acc_buf.size(); i++)
        acc_var += (acc_buf[i] - mean_acc).squaredNorm();
    acc_var /= static_cast<double>(acc_buf.size());

    return acc_var < BLIND_ZUPT_MAX_ACC_VAR && mean_gyr_norm < BLIND_TILT_MAX_GYR &&
           fabs(mean_acc.norm() - G.norm()) < BLIND_TILT_MAX_ACC_DEV;
}

bool Estimator::isGroundStatic(int frame_index) const
{
    if (!isLowDynamic(frame_index))
        return false;
    const vector<Vector3d> &gyr_buf = angular_velocity_buf[frame_index];
    double mean_gyr_norm = 0.0;
    for (size_t i = 0; i < gyr_buf.size(); i++)
        mean_gyr_norm += gyr_buf[i].norm();
    mean_gyr_norm /= static_cast<double>(gyr_buf.size());
    return mean_gyr_norm < BLIND_ZUPT_MAX_GYR;
}

void Estimator::addBlindFactors(ceres::Problem &problem, ceres::LossFunction *loss_function)
{
    if (!BLIND_ENABLE || !USE_IMU || !blind_anchor_valid || !isBlind())
        return;

    const double current_time = Headers[frame_count];
    const double t_blind = blindDuration(current_time);
    const double relax_max = std::max(1.0, BLIND_BIAS_RELAX_MAX);
    const double relax = std::min(relax_max, 1.0 + BLIND_BIAS_RELAX_RATE * t_blind);
    Eigen::Matrix<double, 6, 6> sqrt_info = Eigen::Matrix<double, 6, 6>::Zero();
    sqrt_info.block<3, 3>(0, 0) =
        (1.0 / std::max(1e-6, BLIND_BIAS_ACC_SIGMA * relax)) * Eigen::Matrix3d::Identity();
    sqrt_info.block<3, 3>(3, 3) =
        (1.0 / std::max(1e-6, BLIND_BIAS_GYR_SIGMA * relax)) * Eigen::Matrix3d::Identity();

    for (int i = 0; i <= frame_count; i++)
    {
        BiasPriorFactor *bias_factor = new BiasPriorFactor(blind_ba0, blind_bg0, sqrt_info);
        problem.AddResidualBlock(bias_factor, loss_function, para_SpeedBias[i]);
    }

    if (BLIND_VELOCITY_PRIOR_WEIGHT > 0.0 && blind_v0.allFinite())
    {
        for (int i = 0; i <= frame_count; i++)
        {
            VelocityPriorFactor *velocity_factor =
                new VelocityPriorFactor(blind_v0, BLIND_VELOCITY_PRIOR_WEIGHT);
            problem.AddResidualBlock(velocity_factor, loss_function, para_SpeedBias[i]);
        }
    }

    for (int i = 0; i <= frame_count; i++)
    {
        if (BLIND_TILT_WEIGHT > 0.0 && isLowDynamic(i))
        {
            Vector3d acc_mean = Vector3d::Zero();
            for (size_t k = 0; k < linear_acceleration_buf[i].size(); k++)
                acc_mean += linear_acceleration_buf[i][k];
            acc_mean /= static_cast<double>(linear_acceleration_buf[i].size());
            if (acc_mean.norm() > 1e-3)
            {
                acc_mean.normalize();
                Vector3d g_unit = g.normalized();
                problem.AddResidualBlock(GravityDirectionFactor::Create(g_unit, acc_mean, BLIND_TILT_WEIGHT),
                                         loss_function, para_Pose[i]);
            }
        }

        // Let IMU stationarity decide ZUPT gating. The estimated velocity can
        // already be corrupted once drift starts, so it should not veto recovery.
        if (BLIND_ZUPT_WEIGHT > 0.0 && isGroundStatic(i))
        {
            GroundZuptFactor *zupt_factor = new GroundZuptFactor(BLIND_ZUPT_WEIGHT);
            problem.AddResidualBlock(zupt_factor, loss_function, para_SpeedBias[i]);
        }
    }

    if (BLIND_THRUST_WEIGHT > 0.0)
    {
        for (int i = 0; i < frame_count; i++)
        {
            const double dt = Headers[i + 1] - Headers[i];
            if (dt < BLIND_THRUST_MIN_DT)
                continue;
            double thrust_acc_i = getThrustAcc(Headers[i]);
            double thrust_acc_j = getThrustAcc(Headers[i + 1]);
            if (thrust_acc_i < 0.0 || thrust_acc_j < 0.0)
                continue;
            problem.AddResidualBlock(ThrustDynamicsFactor::Create(dt, thrust_acc_i,
                                                                  thrust_acc_j, g,
                                                                  BLIND_THRUST_WEIGHT),
                                     loss_function,
                                     para_Pose[i], para_SpeedBias[i],
                                     para_Pose[i + 1], para_SpeedBias[i + 1]);
        }
    }
}

void Estimator::addNominalBiasPrior(ceres::Problem &problem, ceres::LossFunction *loss_function)
{
    if (!NOMINAL_BIAS_PRIOR_ENABLE || !USE_IMU || solver_flag != NON_LINEAR || isBlind())
        return;

    const double sigma_acc = std::max(1e-3, NOMINAL_ACC_BIAS_SIGMA);
    Eigen::Matrix<double, 6, 6> sqrt_info = Eigen::Matrix<double, 6, 6>::Zero();
    sqrt_info.block<3, 3>(0, 0) = (1.0 / sigma_acc) * Eigen::Matrix3d::Identity();

    BiasPriorFactor *bias_factor =
        new BiasPriorFactor(Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), sqrt_info);
    problem.AddResidualBlock(bias_factor, nullptr, para_SpeedBias[frame_count]);
}

void Estimator::logDynamicsResiduals()
{
    if (!USE_IMU || solver_flag != NON_LINEAR || frame_count < 2)
        return;

    std::deque<std::pair<double, double>> thrust_buf;
    {
        std::lock_guard<std::mutex> lock(mThrust);
        thrust_buf = thrustBuf;
    }

    int valid = 0;
    int missing_thrust = 0;
    int missing_imu = 0;
    double sq_sum = 0.0;
    double max_norm = 0.0;
    Eigen::Vector3d mean_residual = Eigen::Vector3d::Zero();

    for (int b = 1; b <= frame_count; ++b)
    {
        const int a = b - 1;
        const std::vector<double> &dts = dt_buf[b];
        const std::vector<Eigen::Vector3d> &gyrs = angular_velocity_buf[b];
        if (dts.empty() || dts.size() != gyrs.size())
        {
            ++missing_imu;
            continue;
        }

        Eigen::Matrix3d delta_R = Eigen::Matrix3d::Identity();
        Eigen::Vector3d alpha = Eigen::Vector3d::Zero();
        double sum_dt = 0.0;
        double t = Headers[a];
        bool ok = true;
        for (size_t k = 0; k < dts.size(); ++k)
        {
            const double dt = dts[k];
            if (!std::isfinite(dt) || dt <= 0.0 || dt > 0.05)
            {
                ok = false;
                break;
            }
            const double thrust_acc = sampleThrustAcc(thrust_buf, t + 0.5 * dt);
            if (thrust_acc < 0.0)
            {
                ok = false;
                ++missing_thrust;
                break;
            }
            alpha += delta_R * (thrust_acc * Eigen::Vector3d::UnitZ()) * dt;
            const Eigen::Vector3d gyr_unbias = gyrs[k] - Bgs[a];
            delta_R *= Utility::deltaQ(gyr_unbias * dt).toRotationMatrix();
            t += dt;
            sum_dt += dt;
        }
        if (!ok || sum_dt <= 0.0)
            continue;

        const Eigen::Vector3d pred_dv = Rs[a] * alpha - g * sum_dt;
        const Eigen::Vector3d vio_dv = Vs[b] - Vs[a];
        const Eigen::Vector3d residual = vio_dv - pred_dv;
        const double norm = residual.norm();
        mean_residual += residual;
        sq_sum += norm * norm;
        max_norm = std::max(max_norm, norm);
        ++valid;
    }

    if (valid > 0)
        mean_residual /= static_cast<double>(valid);
    const double rms = valid > 0 ? std::sqrt(sq_sum / static_cast<double>(valid)) : -1.0;

    ROS_WARN_THROTTLE(1.0,
                      "[DYN-LOG] valid=%d missing_thrust=%d missing_imu=%d rms_dv=%.4f max_dv=%.4f mean=(%.4f %.4f %.4f)",
                      valid, missing_thrust, missing_imu, rms, max_norm,
                      mean_residual.x(), mean_residual.y(), mean_residual.z());
}

bool Estimator::clampBlindBiases()
{
    if (!BLIND_ENABLE || !USE_IMU || !isBlind())
        return false;

    const double acc_bias_max = std::max(0.1, BLIND_BIAS_ACC_MAX);
    const double gyr_bias_max = std::max(0.01, BLIND_BIAS_GYR_MAX);
    bool clamped = false;

    for (int i = 0; i <= frame_count; i++)
    {
        if (!Bas[i].allFinite() || !Bgs[i].allFinite())
            return false;

        const double ba_norm = Bas[i].norm();
        if (ba_norm > acc_bias_max)
        {
            Bas[i] *= acc_bias_max / ba_norm;
            clamped = true;
        }

        const double bg_norm = Bgs[i].norm();
        if (bg_norm > gyr_bias_max)
        {
            Bgs[i] *= gyr_bias_max / bg_norm;
            clamped = true;
        }
    }

    if (clamped)
    {
        ROS_WARN("VINS BLIND bias clamped: acc<=%.3f gyr<=%.3f",
                 acc_bias_max, gyr_bias_max);
        for (int i = 0; i <= frame_count; i++)
        {
            if (pre_integrations[i] != nullptr)
                pre_integrations[i]->repropagate(Bas[i], Bgs[i]);
        }
        if (tmp_pre_integration != nullptr)
            tmp_pre_integration->repropagate(Bas[frame_count], Bgs[frame_count]);
    }

    return clamped;
}

void Estimator::optimization()
{
    TicToc t_whole, t_prepare;
    vector2double();

    const bool visual_calibration_ok = (visual_state == VISUAL_HEALTHY);

    ceres::Problem problem;
    ceres::LossFunction *loss_function;
    //loss_function = NULL;
    loss_function = new ceres::HuberLoss(1.0);
    //loss_function = new ceres::CauchyLoss(1.0 / FOCAL_LENGTH);
    //ceres::LossFunction* loss_function = new ceres::HuberLoss(1.0);
    for (int i = 0; i < frame_count + 1; i++)
    {
        ceres::LocalParameterization *local_parameterization = new PoseLocalParameterization();
        problem.AddParameterBlock(para_Pose[i], SIZE_POSE, local_parameterization);
        if(USE_IMU)
            problem.AddParameterBlock(para_SpeedBias[i], SIZE_SPEEDBIAS);
    }
    if(!USE_IMU)
        problem.SetParameterBlockConstant(para_Pose[0]);

    for (int i = 0; i < NUM_OF_CAM; i++)
    {
        ceres::LocalParameterization *local_parameterization = new PoseLocalParameterization();
        problem.AddParameterBlock(para_Ex_Pose[i], SIZE_POSE, local_parameterization);
        if (visual_calibration_ok &&
            ((ESTIMATE_EXTRINSIC && frame_count == WINDOW_SIZE && Vs[0].norm() > 0.2) || openExEstimation))
        {
            //ROS_INFO("estimate extinsic param");
            openExEstimation = 1;
        }
        else
        {
            //ROS_INFO("fix extinsic param");
            problem.SetParameterBlockConstant(para_Ex_Pose[i]);
        }
    }
    problem.AddParameterBlock(para_Td[0], 1);

    if (!visual_calibration_ok || !ESTIMATE_TD || Vs[0].norm() < 0.2)
        problem.SetParameterBlockConstant(para_Td[0]);

    if (last_marginalization_info && last_marginalization_info->valid)
    {
        // construct new marginlization_factor
        MarginalizationFactor *marginalization_factor = new MarginalizationFactor(last_marginalization_info);
        problem.AddResidualBlock(marginalization_factor, NULL,
                                 last_marginalization_parameter_blocks);
    }
    if(USE_IMU)
    {
        for (int i = 0; i < frame_count; i++)
        {
            int j = i + 1;
            if (pre_integrations[j]->sum_dt > 10.0)
                continue;
            IMUFactor* imu_factor = new IMUFactor(pre_integrations[j]);
            problem.AddResidualBlock(imu_factor, NULL, para_Pose[i], para_SpeedBias[i], para_Pose[j], para_SpeedBias[j]);
        }
    }
    addNominalBiasPrior(problem, loss_function);
    addBlindFactors(problem, loss_function);

    const bool visual_stationary = USE_IMU && isLowFlowStationary();
    if (visual_stationary && LOW_FLOW_ZUPT_WEIGHT > 0.0)
    {
        const int window = std::max(1, LOW_FLOW_ZUPT_WINDOW);
        const int start = std::max(0, frame_count - window + 1);
        for (int i = start; i <= frame_count; i++)
        {
            GroundZuptFactor *flow_zupt = new GroundZuptFactor(LOW_FLOW_ZUPT_WEIGHT);
            problem.AddResidualBlock(flow_zupt, loss_function, para_SpeedBias[i]);
        }
        ROS_WARN_THROTTLE(1.0, "VINS low-flow ZUPT active: flow=%.3f tracks=%d keep=%.2f vel=%.2f",
                          latest_frontend_quality_.mean_pixel_flow,
                          latest_frontend_quality_.tracked_after_lk,
                          latest_frontend_quality_.lk_keep_ratio,
                          Vs[frame_count].norm());
    }

    int f_m_cnt = 0;
    int feature_index = -1;
    const std::vector<char> good_feature_mask = selectGoodFeatures();
    const double state_visual_weight =
        isBlind() ? 0.0 :
        (isVisualDegraded() ? std::min(0.25, BLIND_VISUAL_WEIGHT_DEGRADED) : 1.0);
    auto visualScale = [state_visual_weight](double qi, double qj)
    {
        return clampVisualWeight(state_visual_weight * std::sqrt(std::max(0.05, qi) * std::max(0.05, qj)));
    };
    for (auto &it_per_id : f_manager.feature)
        {
            if (!featureReadyForOptimization(it_per_id))
                continue;
     
            ++feature_index;
            if (isBlind())
                continue;
            const bool keep_feature =
                good_feature_mask.empty() ||
                (feature_index >= 0 && feature_index < static_cast<int>(good_feature_mask.size()) &&
                 good_feature_mask[feature_index]);
            if (!keep_feature)
                continue;

            if (!std::isfinite(para_Feature[feature_index][0]))
                para_Feature[feature_index][0] = 1.0 / std::max(INIT_DEPTH, 1.0);
            para_Feature[feature_index][0] =
                std::min(std::max(para_Feature[feature_index][0], kInvDepthMin), kInvDepthMax);
            problem.AddParameterBlock(para_Feature[feature_index], SIZE_FEATURE);
            problem.SetParameterLowerBound(para_Feature[feature_index], 0, kInvDepthMin);
            problem.SetParameterUpperBound(para_Feature[feature_index], 0, kInvDepthMax);

            int imu_i = it_per_id.start_frame, imu_j = imu_i - 1;
            
            Vector3d pts_i = it_per_id.feature_per_frame[0].point;
            const double quality_i = it_per_id.feature_per_frame[0].quality;
            // If a feature has many observations, subsample redundant ones by parallax.
            const int max_obs = 20; // keep at most this many observations per feature
            int obs_n = static_cast<int>(it_per_id.feature_per_frame.size());
            std::vector<char> keep(obs_n, 1);
            if (obs_n > max_obs)
            {
                // compute parallax wrt first observation and select top-k
                std::vector<std::pair<double, int>> scores; scores.reserve(obs_n - 1);
                for (int idx = 1; idx < obs_n; ++idx)
                {
                    Vector3d pts_j = it_per_id.feature_per_frame[idx].point;
                    double parallax = (pts_i.head<2>() - pts_j.head<2>()).norm();
                    scores.emplace_back(parallax, idx);
                }
                std::sort(scores.begin(), scores.end(),
                          [](const std::pair<double, int> &a, const std::pair<double, int> &b)
                          {
                              return a.first > b.first;
                          });
                // keep first (anchor) and top (max_obs-1) others
                std::fill(keep.begin(), keep.end(), 0);
                keep[0] = 1;
                int to_keep = std::min(max_obs - 1, static_cast<int>(scores.size()));
                for (int k = 0; k < to_keep; ++k)
                    keep[scores[k].second] = 1;
            }

            int frame_idx = 0;
            for (auto &it_per_frame : it_per_id.feature_per_frame)
            {
                // skip observation if we decided not to keep it
                if (!keep[frame_idx]) { imu_j++; frame_idx++; continue; }
                imu_j++;
                if (imu_i != imu_j)
                {
                    Vector3d pts_j = it_per_frame.point;
                    const double scale = visualScale(quality_i, it_per_frame.quality);
                    ProjectionTwoFrameOneCamFactor *f_td = new ProjectionTwoFrameOneCamFactor(pts_i, pts_j, it_per_id.feature_per_frame[0].velocity, it_per_frame.velocity,
                                                                     it_per_id.feature_per_frame[0].cur_td, it_per_frame.cur_td, scale);
                    problem.AddResidualBlock(f_td, loss_function, para_Pose[imu_i], para_Pose[imu_j], para_Ex_Pose[0], para_Feature[feature_index], para_Td[0]);
                }

                if(STEREO && it_per_frame.is_stereo)
                {                
                    Vector3d pts_j_right = it_per_frame.pointRight;
                    const double scale = visualScale(quality_i, it_per_frame.qualityRight);
                    if(imu_i != imu_j)
                    {
                        ProjectionTwoFrameTwoCamFactor *f = new ProjectionTwoFrameTwoCamFactor(pts_i, pts_j_right, it_per_id.feature_per_frame[0].velocity, it_per_frame.velocityRight,
                                                                     it_per_id.feature_per_frame[0].cur_td, it_per_frame.cur_td, scale);
                        problem.AddResidualBlock(f, loss_function, para_Pose[imu_i], para_Pose[imu_j], para_Ex_Pose[0], para_Ex_Pose[1], para_Feature[feature_index], para_Td[0]);
                    }
                    else
                    {
                        ProjectionOneFrameTwoCamFactor *f = new ProjectionOneFrameTwoCamFactor(pts_i, pts_j_right, it_per_id.feature_per_frame[0].velocity, it_per_frame.velocityRight,
                                                                     it_per_id.feature_per_frame[0].cur_td, it_per_frame.cur_td, scale);
                        problem.AddResidualBlock(f, loss_function, para_Ex_Pose[0], para_Ex_Pose[1], para_Feature[feature_index], para_Td[0]);
                    }
                   
                }
                f_m_cnt++;
                frame_idx++;
            }
        }

    ROS_DEBUG("visual measurement count: %d", f_m_cnt);
    //printf("prepare for ceres: %f \n", t_prepare.toc());

    ceres::Solver::Options options;

    options.linear_solver_type = isBlind() ? ceres::DENSE_QR : ceres::DENSE_SCHUR;
    options.num_threads = 2;
    options.trust_region_strategy_type = ceres::DOGLEG;
    options.max_num_iterations = NUM_ITERATIONS;
    //options.use_explicit_schur_complement = true;
    //options.minimizer_progress_to_stdout = true;
    //options.use_nonmonotonic_steps = true;
    if (marginalization_flag == MARGIN_OLD)
        options.max_solver_time_in_seconds = SOLVER_TIME * 4.0 / 5.0;
    else
        options.max_solver_time_in_seconds = SOLVER_TIME;
    TicToc t_solver;
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    //cout << summary.BriefReport() << endl;
    ROS_DEBUG("Iterations : %d", static_cast<int>(summary.iterations.size()));
    //printf("solver costs: %f \n", t_solver.toc());

    double2vector();
    clampBlindBiases();
    logDynamicsResiduals();
    //printf("frame_count: %d \n", frame_count);

    if(frame_count < WINDOW_SIZE)
        return;
    
    TicToc t_whole_marginalization;
    if (marginalization_flag == MARGIN_OLD)
    {
        MarginalizationInfo *marginalization_info = new MarginalizationInfo();
        vector2double();

        if (last_marginalization_info && last_marginalization_info->valid)
        {
            vector<int> drop_set;
            for (int i = 0; i < static_cast<int>(last_marginalization_parameter_blocks.size()); i++)
            {
                if (last_marginalization_parameter_blocks[i] == para_Pose[0] ||
                    last_marginalization_parameter_blocks[i] == para_SpeedBias[0])
                    drop_set.push_back(i);
            }
            // construct new marginlization_factor
            MarginalizationFactor *marginalization_factor = new MarginalizationFactor(last_marginalization_info);
            ResidualBlockInfo *residual_block_info = new ResidualBlockInfo(marginalization_factor, NULL,
                                                                           last_marginalization_parameter_blocks,
                                                                           drop_set);
            marginalization_info->addResidualBlockInfo(residual_block_info);
        }

        if(USE_IMU)
        {
            if (pre_integrations[1]->sum_dt < 10.0)
            {
                IMUFactor* imu_factor = new IMUFactor(pre_integrations[1]);
                ResidualBlockInfo *residual_block_info = new ResidualBlockInfo(imu_factor, NULL,
                                                                           vector<double *>{para_Pose[0], para_SpeedBias[0], para_Pose[1], para_SpeedBias[1]},
                                                                           vector<int>{0, 1});
                marginalization_info->addResidualBlockInfo(residual_block_info);
            }
        }

        {
            int feature_index = -1;
            const std::vector<char> good_feature_mask = selectGoodFeatures();
            for (auto &it_per_id : f_manager.feature)
            {
                if (!featureReadyForOptimization(it_per_id))
                    continue;

                ++feature_index;
                const bool keep_feature =
                    good_feature_mask.empty() ||
                    (feature_index >= 0 && feature_index < static_cast<int>(good_feature_mask.size()) &&
                     good_feature_mask[feature_index]);
                if (!keep_feature)
                    continue;

                int imu_i = it_per_id.start_frame, imu_j = imu_i - 1;
                if (imu_i != 0)
                    continue;

                Vector3d pts_i = it_per_id.feature_per_frame[0].point;
                const double quality_i = it_per_id.feature_per_frame[0].quality;

                for (auto &it_per_frame : it_per_id.feature_per_frame)
                {
                    imu_j++;
                    if(imu_i != imu_j)
                    {
                        Vector3d pts_j = it_per_frame.point;
                        const double scale = visualScale(quality_i, it_per_frame.quality);
                        ProjectionTwoFrameOneCamFactor *f_td = new ProjectionTwoFrameOneCamFactor(pts_i, pts_j, it_per_id.feature_per_frame[0].velocity, it_per_frame.velocity,
                                                                          it_per_id.feature_per_frame[0].cur_td, it_per_frame.cur_td, scale);
                        ResidualBlockInfo *residual_block_info = new ResidualBlockInfo(f_td, loss_function,
                                                                                        vector<double *>{para_Pose[imu_i], para_Pose[imu_j], para_Ex_Pose[0], para_Feature[feature_index], para_Td[0]},
                                                                                        vector<int>{0, 3});
                        marginalization_info->addResidualBlockInfo(residual_block_info);
                    }
                    if(STEREO && it_per_frame.is_stereo)
                    {
                        Vector3d pts_j_right = it_per_frame.pointRight;
                        const double scale = visualScale(quality_i, it_per_frame.qualityRight);
                        if(imu_i != imu_j)
                        {
                            ProjectionTwoFrameTwoCamFactor *f = new ProjectionTwoFrameTwoCamFactor(pts_i, pts_j_right, it_per_id.feature_per_frame[0].velocity, it_per_frame.velocityRight,
                                                                          it_per_id.feature_per_frame[0].cur_td, it_per_frame.cur_td, scale);
                            ResidualBlockInfo *residual_block_info = new ResidualBlockInfo(f, loss_function,
                                                                                           vector<double *>{para_Pose[imu_i], para_Pose[imu_j], para_Ex_Pose[0], para_Ex_Pose[1], para_Feature[feature_index], para_Td[0]},
                                                                                           vector<int>{0, 4});
                            marginalization_info->addResidualBlockInfo(residual_block_info);
                        }
                        else
                        {
                            ProjectionOneFrameTwoCamFactor *f = new ProjectionOneFrameTwoCamFactor(pts_i, pts_j_right, it_per_id.feature_per_frame[0].velocity, it_per_frame.velocityRight,
                                                                          it_per_id.feature_per_frame[0].cur_td, it_per_frame.cur_td, scale);
                            ResidualBlockInfo *residual_block_info = new ResidualBlockInfo(f, loss_function,
                                                                                           vector<double *>{para_Ex_Pose[0], para_Ex_Pose[1], para_Feature[feature_index], para_Td[0]},
                                                                                           vector<int>{2});
                            marginalization_info->addResidualBlockInfo(residual_block_info);
                        }
                    }
                }
            }
        }

        TicToc t_pre_margin;
        marginalization_info->preMarginalize();
        ROS_DEBUG("pre marginalization %f ms", t_pre_margin.toc());
        
        TicToc t_margin;
        marginalization_info->marginalize();
        ROS_DEBUG("marginalization %f ms", t_margin.toc());

        std::unordered_map<long, double *> addr_shift;
        for (int i = 1; i <= WINDOW_SIZE; i++)
        {
            addr_shift[reinterpret_cast<long>(para_Pose[i])] = para_Pose[i - 1];
            if(USE_IMU)
                addr_shift[reinterpret_cast<long>(para_SpeedBias[i])] = para_SpeedBias[i - 1];
        }
        for (int i = 0; i < NUM_OF_CAM; i++)
            addr_shift[reinterpret_cast<long>(para_Ex_Pose[i])] = para_Ex_Pose[i];

        addr_shift[reinterpret_cast<long>(para_Td[0])] = para_Td[0];

        vector<double *> parameter_blocks = marginalization_info->getParameterBlocks(addr_shift);

        if (last_marginalization_info)
            delete last_marginalization_info;
        last_marginalization_info = marginalization_info;
        last_marginalization_parameter_blocks = parameter_blocks;
        
    }
    else
    {
        if (last_marginalization_info &&
            std::count(std::begin(last_marginalization_parameter_blocks), std::end(last_marginalization_parameter_blocks), para_Pose[WINDOW_SIZE - 1]))
        {

            MarginalizationInfo *marginalization_info = new MarginalizationInfo();
            vector2double();
            if (last_marginalization_info && last_marginalization_info->valid)
            {
                vector<int> drop_set;
                for (int i = 0; i < static_cast<int>(last_marginalization_parameter_blocks.size()); i++)
                {
                    ROS_ASSERT(last_marginalization_parameter_blocks[i] != para_SpeedBias[WINDOW_SIZE - 1]);
                    if (last_marginalization_parameter_blocks[i] == para_Pose[WINDOW_SIZE - 1])
                        drop_set.push_back(i);
                }
                // construct new marginlization_factor
                MarginalizationFactor *marginalization_factor = new MarginalizationFactor(last_marginalization_info);
                ResidualBlockInfo *residual_block_info = new ResidualBlockInfo(marginalization_factor, NULL,
                                                                               last_marginalization_parameter_blocks,
                                                                               drop_set);

                marginalization_info->addResidualBlockInfo(residual_block_info);
            }

            TicToc t_pre_margin;
            ROS_DEBUG("begin marginalization");
            marginalization_info->preMarginalize();
            ROS_DEBUG("end pre marginalization, %f ms", t_pre_margin.toc());

            TicToc t_margin;
            ROS_DEBUG("begin marginalization");
            marginalization_info->marginalize();
            ROS_DEBUG("end marginalization, %f ms", t_margin.toc());
            
            std::unordered_map<long, double *> addr_shift;
            for (int i = 0; i <= WINDOW_SIZE; i++)
            {
                if (i == WINDOW_SIZE - 1)
                    continue;
                else if (i == WINDOW_SIZE)
                {
                    addr_shift[reinterpret_cast<long>(para_Pose[i])] = para_Pose[i - 1];
                    if(USE_IMU)
                        addr_shift[reinterpret_cast<long>(para_SpeedBias[i])] = para_SpeedBias[i - 1];
                }
                else
                {
                    addr_shift[reinterpret_cast<long>(para_Pose[i])] = para_Pose[i];
                    if(USE_IMU)
                        addr_shift[reinterpret_cast<long>(para_SpeedBias[i])] = para_SpeedBias[i];
                }
            }
            for (int i = 0; i < NUM_OF_CAM; i++)
                addr_shift[reinterpret_cast<long>(para_Ex_Pose[i])] = para_Ex_Pose[i];

            addr_shift[reinterpret_cast<long>(para_Td[0])] = para_Td[0];

            
            vector<double *> parameter_blocks = marginalization_info->getParameterBlocks(addr_shift);
            if (last_marginalization_info)
                delete last_marginalization_info;
            last_marginalization_info = marginalization_info;
            last_marginalization_parameter_blocks = parameter_blocks;
            
        }
    }
    //printf("whole marginalization costs: %f \n", t_whole_marginalization.toc());
    //printf("whole time for ceres: %f \n", t_whole.toc());
}

void Estimator::slideWindow()
{
    TicToc t_margin;
    if (marginalization_flag == MARGIN_OLD)
    {
        double t_0 = Headers[0];
        back_R0 = Rs[0];
        back_P0 = Ps[0];
        if (frame_count == WINDOW_SIZE)
        {
            for (int i = 0; i < WINDOW_SIZE; i++)
            {
                Headers[i] = Headers[i + 1];
                Rs[i].swap(Rs[i + 1]);
                Ps[i].swap(Ps[i + 1]);
                if(USE_IMU)
                {
                    std::swap(pre_integrations[i], pre_integrations[i + 1]);

                    dt_buf[i].swap(dt_buf[i + 1]);
                    linear_acceleration_buf[i].swap(linear_acceleration_buf[i + 1]);
                    angular_velocity_buf[i].swap(angular_velocity_buf[i + 1]);

                    Vs[i].swap(Vs[i + 1]);
                    Bas[i].swap(Bas[i + 1]);
                    Bgs[i].swap(Bgs[i + 1]);
                }
            }
            Headers[WINDOW_SIZE] = Headers[WINDOW_SIZE - 1];
            Ps[WINDOW_SIZE] = Ps[WINDOW_SIZE - 1];
            Rs[WINDOW_SIZE] = Rs[WINDOW_SIZE - 1];

            if(USE_IMU)
            {
                Vs[WINDOW_SIZE] = Vs[WINDOW_SIZE - 1];
                Bas[WINDOW_SIZE] = Bas[WINDOW_SIZE - 1];
                Bgs[WINDOW_SIZE] = Bgs[WINDOW_SIZE - 1];

                delete pre_integrations[WINDOW_SIZE];
                pre_integrations[WINDOW_SIZE] = new IntegrationBase{acc_0, gyr_0, Bas[WINDOW_SIZE], Bgs[WINDOW_SIZE]};

                dt_buf[WINDOW_SIZE].clear();
                linear_acceleration_buf[WINDOW_SIZE].clear();
                angular_velocity_buf[WINDOW_SIZE].clear();
            }

            if (true || solver_flag == INITIAL)
            {
                map<double, ImageFrame>::iterator it_0;
                it_0 = all_image_frame.find(t_0);
                if (it_0 != all_image_frame.end())
                {
                    if (it_0->second.pre_integration != nullptr)
                    {
                        delete it_0->second.pre_integration;
                        it_0->second.pre_integration = nullptr;
                    }
                    all_image_frame.erase(all_image_frame.begin(), it_0);
                }
                else
                {
                    ROS_WARN("slideWindow missing image frame %.9f during marginalization", t_0);
                }
            }
            slideWindowOld();
        }
    }
    else
    {
        if (frame_count == WINDOW_SIZE)
        {
            Headers[frame_count - 1] = Headers[frame_count];
            Ps[frame_count - 1] = Ps[frame_count];
            Rs[frame_count - 1] = Rs[frame_count];

            if(USE_IMU)
            {
                for (unsigned int i = 0; i < dt_buf[frame_count].size(); i++)
                {
                    double tmp_dt = dt_buf[frame_count][i];
                    Vector3d tmp_linear_acceleration = linear_acceleration_buf[frame_count][i];
                    Vector3d tmp_angular_velocity = angular_velocity_buf[frame_count][i];

                    pre_integrations[frame_count - 1]->push_back(tmp_dt, tmp_linear_acceleration, tmp_angular_velocity);

                    dt_buf[frame_count - 1].push_back(tmp_dt);
                    linear_acceleration_buf[frame_count - 1].push_back(tmp_linear_acceleration);
                    angular_velocity_buf[frame_count - 1].push_back(tmp_angular_velocity);
                }

                Vs[frame_count - 1] = Vs[frame_count];
                Bas[frame_count - 1] = Bas[frame_count];
                Bgs[frame_count - 1] = Bgs[frame_count];

                delete pre_integrations[WINDOW_SIZE];
                pre_integrations[WINDOW_SIZE] = new IntegrationBase{acc_0, gyr_0, Bas[WINDOW_SIZE], Bgs[WINDOW_SIZE]};

                dt_buf[WINDOW_SIZE].clear();
                linear_acceleration_buf[WINDOW_SIZE].clear();
                angular_velocity_buf[WINDOW_SIZE].clear();
            }
            slideWindowNew();
        }
    }
}

void Estimator::slideWindowNew()
{
    sum_of_front++;
    f_manager.removeFront(frame_count);
}

void Estimator::slideWindowOld()
{
    sum_of_back++;

    bool shift_depth = solver_flag == NON_LINEAR ? true : false;
    if (shift_depth)
    {
        Matrix3d R0, R1;
        Vector3d P0, P1;
        R0 = back_R0 * ric[0];
        R1 = Rs[0] * ric[0];
        P0 = back_P0 + back_R0 * tic[0];
        P1 = Ps[0] + Rs[0] * tic[0];
        f_manager.removeBackShiftDepth(R0, P0, R1, P1);
    }
    else
        f_manager.removeBack();
}


void Estimator::getPoseInWorldFrame(Eigen::Matrix4d &T)
{
    T = Eigen::Matrix4d::Identity();
    T.block<3, 3>(0, 0) = Rs[frame_count];
    T.block<3, 1>(0, 3) = Ps[frame_count];
}

void Estimator::getPoseInWorldFrame(int index, Eigen::Matrix4d &T)
{
    T = Eigen::Matrix4d::Identity();
    T.block<3, 3>(0, 0) = Rs[index];
    T.block<3, 1>(0, 3) = Ps[index];
}

void Estimator::predictPtsInNextFrame()
{
    if(frame_count < 2)
        return;
    Eigen::Matrix4d curT, prevT, nextT;
    getPoseInWorldFrame(curT);
    getPoseInWorldFrame(frame_count - 1, prevT);
    nextT = curT * (prevT.inverse() * curT);

    map<int, Eigen::Vector3d> predictPts;

    for (auto &it_per_id : f_manager.feature)
    {
        if(it_per_id.estimated_depth > 0)
        {
            int firstIndex = it_per_id.start_frame;
            int lastIndex = it_per_id.start_frame + it_per_id.feature_per_frame.size() - 1;
            //printf("cur frame index  %d last frame index %d\n", frame_count, lastIndex);
            if((int)it_per_id.feature_per_frame.size() >= 2 && lastIndex == frame_count)
            {
                double depth = it_per_id.estimated_depth;
                Vector3d pts_j = ric[0] * (depth * it_per_id.feature_per_frame[0].point) + tic[0];
                Vector3d pts_w = Rs[firstIndex] * pts_j + Ps[firstIndex];
                Vector3d pts_local = nextT.block<3, 3>(0, 0).transpose() * (pts_w - nextT.block<3, 1>(0, 3));
                Vector3d pts_cam = ric[0].transpose() * (pts_local - tic[0]);
                int ptsIndex = it_per_id.feature_id;
                predictPts[ptsIndex] = pts_cam;
            }
        }
    }
    featureTracker.setPrediction(predictPts);
    //printf("estimator output %d predict pts\n",(int)predictPts.size());
}

double Estimator::reprojectionError(const Matrix3d &Ri, const Vector3d &Pi, const Matrix3d &rici, const Vector3d &tici,
                                 const Matrix3d &Rj, const Vector3d &Pj, const Matrix3d &ricj, const Vector3d &ticj, 
                                 double depth, const Vector3d &uvi, const Vector3d &uvj) const
{
    if (!std::isfinite(depth) || depth <= 0.0)
        return std::numeric_limits<double>::infinity();
    Vector3d pts_w = Ri * (rici * (depth * uvi) + tici) + Pi;
    Vector3d pts_cj = ricj.transpose() * (Rj.transpose() * (pts_w - Pj) - ticj);
    if (!pts_cj.allFinite() || pts_cj.z() <= 1e-3)
        return std::numeric_limits<double>::infinity();
    Vector2d residual = (pts_cj / pts_cj.z()).head<2>() - uvj.head<2>();
    if (!residual.allFinite())
        return std::numeric_limits<double>::infinity();
    double rx = residual.x();
    double ry = residual.y();
    return sqrt(rx * rx + ry * ry);
}

void Estimator::outliersRejection(set<int> &removeIndex)
{
    //return;
    int feature_index = -1;
    for (auto &it_per_id : f_manager.feature)
    {
        double err = 0;
        int errCnt = 0;
        if (!featureReadyForOptimization(it_per_id))
            continue;
        feature_index ++;
        int imu_i = it_per_id.start_frame, imu_j = imu_i - 1;
        Vector3d pts_i = it_per_id.feature_per_frame[0].point;
        double depth = it_per_id.estimated_depth;
        for (auto &it_per_frame : it_per_id.feature_per_frame)
        {
            imu_j++;
            if (imu_i != imu_j)
            {
                Vector3d pts_j = it_per_frame.point;             
                double tmp_error = reprojectionError(Rs[imu_i], Ps[imu_i], ric[0], tic[0], 
                                                    Rs[imu_j], Ps[imu_j], ric[0], tic[0],
                                                    depth, pts_i, pts_j);
                err += tmp_error;
                errCnt++;
                //printf("tmp_error %f\n", FOCAL_LENGTH / 1.5 * tmp_error);
            }
            // need to rewrite projecton factor.........
            if(STEREO && it_per_frame.is_stereo)
            {
                
                Vector3d pts_j_right = it_per_frame.pointRight;
                if(imu_i != imu_j)
                {            
                    double tmp_error = reprojectionError(Rs[imu_i], Ps[imu_i], ric[0], tic[0], 
                                                        Rs[imu_j], Ps[imu_j], ric[1], tic[1],
                                                        depth, pts_i, pts_j_right);
                    err += tmp_error;
                    errCnt++;
                    //printf("tmp_error %f\n", FOCAL_LENGTH / 1.5 * tmp_error);
                }
                else
                {
                    double tmp_error = reprojectionError(Rs[imu_i], Ps[imu_i], ric[0], tic[0], 
                                                        Rs[imu_j], Ps[imu_j], ric[1], tic[1],
                                                        depth, pts_i, pts_j_right);
                    err += tmp_error;
                    errCnt++;
                    //printf("tmp_error %f\n", FOCAL_LENGTH / 1.5 * tmp_error);
                }       
            }
        }
        if (errCnt == 0)
        {
            removeIndex.insert(it_per_id.feature_id);
            continue;
        }
        double ave_err = err / errCnt;
        if (!std::isfinite(ave_err))
        {
            removeIndex.insert(it_per_id.feature_id);
            continue;
        }
        if(ave_err * FOCAL_LENGTH > 3)
            removeIndex.insert(it_per_id.feature_id);

    }
}

void Estimator::fastPredictIMU(double t, Eigen::Vector3d linear_acceleration, Eigen::Vector3d angular_velocity)
{
    if (!std::isfinite(latest_time) || latest_time <= 0.0)
    {
        latest_time = t;
        latest_acc_0 = linear_acceleration;
        latest_gyr_0 = angular_velocity;
        return;
    }
    double dt = t - latest_time;
    if (!std::isfinite(dt) || dt <= 0.0 || dt > 0.05)
    {
        latest_time = t;
        latest_acc_0 = linear_acceleration;
        latest_gyr_0 = angular_velocity;
        ROS_WARN_THROTTLE(1.0, "skip abnormal propagate dt: %.6f at t=%.6f", dt, t);
        return;
    }
    latest_time = t;
    Eigen::Vector3d un_acc_0 = latest_Q * (latest_acc_0 - latest_Ba) - g;
    Eigen::Vector3d un_gyr = 0.5 * (latest_gyr_0 + angular_velocity) - latest_Bg;
    latest_Q = latest_Q * Utility::deltaQ(un_gyr * dt);
    latest_Q.normalize();
    if (isLowFlowStationary())
    {
        if (low_flow_lock_active_)
            latest_P = low_flow_lock_P_;
        latest_V.setZero();
        latest_acc_0 = linear_acceleration;
        latest_gyr_0 = angular_velocity;
        return;
    }
    Eigen::Vector3d un_acc_1 = latest_Q * (linear_acceleration - latest_Ba) - g;
    Eigen::Vector3d un_acc = 0.5 * (un_acc_0 + un_acc_1);
    latest_P = latest_P + dt * latest_V + 0.5 * dt * dt * un_acc;
    latest_V = latest_V + dt * un_acc;
    latest_acc_0 = linear_acceleration;
    latest_gyr_0 = angular_velocity;
}

void Estimator::updateLatestStates()
{
    mPropagate.lock();
    latest_time = Headers[frame_count] + td;
    latest_P = Ps[frame_count];
    latest_Q = Rs[frame_count];
    latest_V = Vs[frame_count];
    if (isLowFlowStationary())
    {
        if (low_flow_lock_active_)
            latest_P = low_flow_lock_P_;
        latest_V.setZero();
    }
    latest_Ba = Bas[frame_count];
    latest_Bg = Bgs[frame_count];
    latest_acc_0 = acc_0;
    latest_gyr_0 = gyr_0;
    mBuf.lock();
    queue<pair<double, Eigen::Vector3d>> tmp_accBuf = accBuf;
    queue<pair<double, Eigen::Vector3d>> tmp_gyrBuf = gyrBuf;
    mBuf.unlock();
    while(!tmp_accBuf.empty())
    {
        double t = tmp_accBuf.front().first;
        Eigen::Vector3d acc = tmp_accBuf.front().second;
        Eigen::Vector3d gyr = tmp_gyrBuf.front().second;
        fastPredictIMU(t, acc, gyr);
        tmp_accBuf.pop();
        tmp_gyrBuf.pop();
    }
    mPropagate.unlock();
}
