/*******************************************************
 * Zero Velocity Update (ZUPT) Factor for VINS-Fusion
 *
 * 残差: r = weight * V_i, 维度 3
 * 参数块: para_SpeedBias[i], 维度 9 (vx,vy,vz, bax,bay,baz, bgx,bgy,bgz)
 * 只对速度部分(前3维)起作用, 对 bias 部分雅可比为 0
 *******************************************************/

#pragma once

#include <ceres/ceres.h>
#include <Eigen/Dense>

class ZeroVelocityFactor : public ceres::SizedCostFunction<3, 9>
{
public:
    ZeroVelocityFactor() = delete;
    explicit ZeroVelocityFactor(double weight) : weight_(weight) {}

    virtual bool Evaluate(double const *const *parameters,
                          double *residuals,
                          double **jacobians) const
    {
        // para_SpeedBias 前3维就是速度
        Eigen::Map<const Eigen::Vector3d> V(parameters[0]);
        Eigen::Map<Eigen::Vector3d> r(residuals);
        r = weight_ * V;

        if (jacobians && jacobians[0])
        {
            // 残差对 9 维参数块的雅可比, 行主序
            Eigen::Map<Eigen::Matrix<double, 3, 9, Eigen::RowMajor>> J(jacobians[0]);
            J.setZero();
            J.block<3, 3>(0, 0) = weight_ * Eigen::Matrix3d::Identity();
        }
        return true;
    }

private:
    double weight_;
};
