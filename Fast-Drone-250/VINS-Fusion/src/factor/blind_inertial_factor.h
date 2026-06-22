/*******************************************************
 * Blind inertial soft factors for short visual outages.
 *******************************************************/

#pragma once

#include <ceres/ceres.h>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>

class BiasPriorFactor : public ceres::SizedCostFunction<6, 9>
{
  public:
    BiasPriorFactor(const Eigen::Vector3d &ba0,
                    const Eigen::Vector3d &bg0,
                    const Eigen::Matrix<double, 6, 6> &sqrt_info)
        : ba0_(ba0), bg0_(bg0), sqrt_info_(sqrt_info)
    {
    }

    virtual bool Evaluate(double const *const *parameters,
                          double *residuals,
                          double **jacobians) const
    {
        Eigen::Map<const Eigen::Vector3d> ba(parameters[0] + 3);
        Eigen::Map<const Eigen::Vector3d> bg(parameters[0] + 6);
        Eigen::Map<Eigen::Matrix<double, 6, 1>> r(residuals);

        r.head<3>() = ba - ba0_;
        r.tail<3>() = bg - bg0_;
        r = sqrt_info_ * r;

        if (jacobians && jacobians[0])
        {
            Eigen::Map<Eigen::Matrix<double, 6, 9, Eigen::RowMajor>> J(jacobians[0]);
            J.setZero();
            J.block<3, 3>(0, 3).setIdentity();
            J.block<3, 3>(3, 6).setIdentity();
            J = sqrt_info_ * J;
        }
        return true;
    }

  private:
    Eigen::Vector3d ba0_;
    Eigen::Vector3d bg0_;
    Eigen::Matrix<double, 6, 6> sqrt_info_;
};

class GroundZuptFactor : public ceres::SizedCostFunction<3, 9>
{
  public:
    explicit GroundZuptFactor(double weight) : weight_(weight)
    {
    }

    virtual bool Evaluate(double const *const *parameters,
                          double *residuals,
                          double **jacobians) const
    {
        Eigen::Map<const Eigen::Vector3d> v(parameters[0]);
        Eigen::Map<Eigen::Vector3d> r(residuals);
        r = weight_ * v;

        if (jacobians && jacobians[0])
        {
            Eigen::Map<Eigen::Matrix<double, 3, 9, Eigen::RowMajor>> J(jacobians[0]);
            J.setZero();
            J.block<3, 3>(0, 0) = weight_ * Eigen::Matrix3d::Identity();
        }
        return true;
    }

  private:
    double weight_;
};

class VelocityPriorFactor : public ceres::SizedCostFunction<3, 9>
{
  public:
    VelocityPriorFactor(const Eigen::Vector3d &v0, double weight)
        : v0_(v0), weight_(weight)
    {
    }

    virtual bool Evaluate(double const *const *parameters,
                          double *residuals,
                          double **jacobians) const
    {
        Eigen::Map<const Eigen::Vector3d> v(parameters[0]);
        Eigen::Map<Eigen::Vector3d> r(residuals);
        r = weight_ * (v - v0_);

        if (jacobians && jacobians[0])
        {
            Eigen::Map<Eigen::Matrix<double, 3, 9, Eigen::RowMajor>> J(jacobians[0]);
            J.setZero();
            J.block<3, 3>(0, 0) = weight_ * Eigen::Matrix3d::Identity();
        }
        return true;
    }

  private:
    Eigen::Vector3d v0_;
    double weight_;
};

struct GravityDirectionFactor
{
    GravityDirectionFactor(const Eigen::Vector3d &g_world_unit,
                           const Eigen::Vector3d &acc_body_unit,
                           double weight)
        : g_world_unit_(g_world_unit), acc_body_unit_(acc_body_unit), weight_(weight)
    {
    }

    template <typename T>
    bool operator()(const T *const pose, T *residuals) const
    {
        Eigen::Quaternion<T> q(pose[6], pose[3], pose[4], pose[5]);
        Eigen::Matrix<T, 3, 1> g_world(T(g_world_unit_.x()), T(g_world_unit_.y()), T(g_world_unit_.z()));
        Eigen::Matrix<T, 3, 1> acc_body(T(acc_body_unit_.x()), T(acc_body_unit_.y()), T(acc_body_unit_.z()));
        Eigen::Matrix<T, 3, 1> g_body = q.conjugate() * g_world;
        Eigen::Matrix<T, 3, 1> r = g_body.cross(acc_body);
        residuals[0] = T(weight_) * r.x();
        residuals[1] = T(weight_) * r.y();
        residuals[2] = T(weight_) * r.z();
        return true;
    }

    static ceres::CostFunction *Create(const Eigen::Vector3d &g_world_unit,
                                       const Eigen::Vector3d &acc_body_unit,
                                       double weight)
    {
        return new ceres::AutoDiffCostFunction<GravityDirectionFactor, 3, 7>(
            new GravityDirectionFactor(g_world_unit, acc_body_unit, weight));
    }

    Eigen::Vector3d g_world_unit_;
    Eigen::Vector3d acc_body_unit_;
    double weight_;
};

struct ThrustDynamicsFactor
{
    ThrustDynamicsFactor(double dt,
                         double thrust_acc_i,
                         double thrust_acc_j,
                         const Eigen::Vector3d &g_world,
                         double weight)
        : dt_(dt),
          thrust_acc_i_(thrust_acc_i),
          thrust_acc_j_(thrust_acc_j),
          g_world_(g_world),
          weight_(weight)
    {
    }

    template <typename T>
    bool operator()(const T *const pose_i,
                    const T *const speedbias_i,
                    const T *const pose_j,
                    const T *const speedbias_j,
                    T *residuals) const
    {
        Eigen::Quaternion<T> qi(pose_i[6], pose_i[3], pose_i[4], pose_i[5]);
        Eigen::Quaternion<T> qj(pose_j[6], pose_j[3], pose_j[4], pose_j[5]);

        Eigen::Matrix<T, 3, 1> vi(speedbias_i[0], speedbias_i[1], speedbias_i[2]);
        Eigen::Matrix<T, 3, 1> vj(speedbias_j[0], speedbias_j[1], speedbias_j[2]);
        Eigen::Matrix<T, 3, 1> ez(T(0), T(0), T(1));
        Eigen::Matrix<T, 3, 1> g(T(g_world_.x()), T(g_world_.y()), T(g_world_.z()));

        Eigen::Matrix<T, 3, 1> thrust_world =
            T(0.5) * (qi * (T(thrust_acc_i_) * ez) + qj * (T(thrust_acc_j_) * ez));
        Eigen::Matrix<T, 3, 1> predicted_dv = (thrust_world - g) * T(dt_);
        Eigen::Matrix<T, 3, 1> r = (vj - vi) - predicted_dv;

        residuals[0] = T(weight_) * r.x();
        residuals[1] = T(weight_) * r.y();
        residuals[2] = T(weight_) * r.z();
        return true;
    }

    static ceres::CostFunction *Create(double dt,
                                       double thrust_acc_i,
                                       double thrust_acc_j,
                                       const Eigen::Vector3d &g_world,
                                       double weight)
    {
        return new ceres::AutoDiffCostFunction<ThrustDynamicsFactor, 3, 7, 9, 7, 9>(
            new ThrustDynamicsFactor(dt, thrust_acc_i, thrust_acc_j, g_world, weight));
    }

    double dt_;
    double thrust_acc_i_;
    double thrust_acc_j_;
    Eigen::Vector3d g_world_;
    double weight_;
};
