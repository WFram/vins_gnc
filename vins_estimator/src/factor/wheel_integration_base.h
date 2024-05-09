/*******************************************************
 * Copyright (C) 2019, Aerial Robotics Group, Hong Kong University of Science and Technology
 *
 * This file is part of VINS.
 *
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *******************************************************/

#pragma once

#include "../estimator/parameters.h"
#include "../utility/sophus_utils.hpp"
#include "../utility/utility.h"

#include <ceres/ceres.h>
using namespace Eigen;

class WheelIntegrationBase {
 public:
  WheelIntegrationBase() = delete;

  WheelIntegrationBase(const Eigen::Vector3d &input_linear_velocity_data,
                       const Eigen::Vector3d &input_angular_velocity_data, const double &input_sx_data,
                       const double &input_sy_data, const double &input_sw_data, const double &input_time_offset_data)
      : latest_linear_velocity{input_linear_velocity_data},
        latest_angular_velocity{input_angular_velocity_data},
        linearized_linear_velocity{input_linear_velocity_data},
        linearized_angular_velocity{input_angular_velocity_data},
        linearized_sx{input_sx_data},
        linearized_sy{input_sy_data},
        linearized_sw{input_sw_data},
        linearized_td{input_time_offset_data},
        jacobian{Eigen::Matrix<double, 6, 3>::Zero()},
        covariance{Eigen::Matrix<double, 6, 6>::Zero()},
        sum_dt{0.0},
        preintegration_alpha{Eigen::Vector3d::Zero()},
        preintegration_gamma{Eigen::Quaterniond::Identity()}

  {
    noise = Eigen::Matrix<double, 12, 12>::Zero();
    noise.block<2, 2>(0, 0) = (WO_LIN_VEL_XY_N * WO_LIN_VEL_XY_N) * Eigen::Matrix2d::Identity();
    noise.block<1, 1>(2, 2) = (WO_LIN_VEL_Z_N * WO_LIN_VEL_Z_N) * Eigen::Matrix<double, 1, 1>::Identity();
    noise.block<2, 2>(3, 3) = (WO_ANG_VEL_XY_N * WO_ANG_VEL_XY_N) * Eigen::Matrix2d::Identity();
    noise.block<1, 1>(5, 5) = (WO_ANG_VEL_Z_N * WO_ANG_VEL_Z_N) * Eigen::Matrix<double, 1, 1>::Identity();
    noise.block<2, 2>(6, 6) = (WO_LIN_VEL_XY_N * WO_LIN_VEL_XY_N) * Eigen::Matrix2d::Identity();
    noise.block<1, 1>(8, 8) = (WO_LIN_VEL_Z_N * WO_LIN_VEL_Z_N) * Eigen::Matrix<double, 1, 1>::Identity();
    noise.block<2, 2>(9, 9) = (WO_ANG_VEL_XY_N * WO_ANG_VEL_XY_N) * Eigen::Matrix2d::Identity();
    noise.block<1, 1>(11, 11) = (WO_ANG_VEL_Z_N * WO_ANG_VEL_Z_N) * Eigen::Matrix<double, 1, 1>::Identity();
  }

  void push_back(double dt, const Eigen::Vector3d &linear_velocity, const Eigen::Vector3d &angular_velocity) {
    dt_buffer.push_back(dt);
    linear_velocity_buffer.push_back(linear_velocity);
    angular_velocity_buffer.push_back(angular_velocity);
    propagate(dt, linear_velocity, angular_velocity);
  }

  void repropagate(const double &input_sx, const double &input_sy, const double &input_sw) {
    sum_dt = 0.0;
    latest_linear_velocity = linearized_linear_velocity;
    latest_angular_velocity = linearized_angular_velocity;
    preintegration_alpha.setZero();
    preintegration_gamma.setIdentity();
    linearized_sx = input_sx;
    linearized_sy = input_sy;
    linearized_sw = input_sw;
    jacobian.setZero();
    covariance.setZero();
    for (int i = 0; i < static_cast<int>(dt_buffer.size()); i++)
      propagate(dt_buffer[i], linear_velocity_buffer[i], angular_velocity_buffer[i]);
  }

  void midPointIntegration() {
    Eigen::Matrix3d sv = Eigen::Vector3d(linearized_sx, linearized_sy, 1).asDiagonal();
    Vector3d left_linear_velocity = preintegration_gamma * sv * latest_linear_velocity;
    Vector3d mid_angular_velocity = 0.5 * linearized_sw * (latest_angular_velocity + input_angular_velocity);
    Eigen::Quaterniond orientation_increment = Quaterniond(
        1, mid_angular_velocity(0) * dt / 2, mid_angular_velocity(1) * dt / 2, mid_angular_velocity(2) * dt / 2);
    preintegration_gamma = preintegration_gamma * orientation_increment;
    Vector3d right_linear_velocity = preintegration_gamma * sv * input_linear_velocity;
    Vector3d mid_linear_velocity = 0.5 * (left_linear_velocity + right_linear_velocity);
    preintegration_alpha = preintegration_alpha + mid_linear_velocity * dt;

    Vector3d scaled_left_linear_velocity = sv * latest_linear_velocity;
    Vector3d scaled_right_linear_velocity = sv * input_linear_velocity;
    Matrix3d R_w, R_v0, R_v1;

    R_w << 0, -mid_angular_velocity(2), mid_angular_velocity(1), mid_angular_velocity(2), 0, -mid_angular_velocity(0),
        -mid_angular_velocity(1), mid_angular_velocity(0), 0;
    R_v0 << 0, -scaled_left_linear_velocity(2), scaled_left_linear_velocity(1), scaled_left_linear_velocity(2), 0,
        -scaled_left_linear_velocity(0), -scaled_left_linear_velocity(1), scaled_left_linear_velocity(0), 0;
    R_v1 << 0, -scaled_right_linear_velocity(2), scaled_right_linear_velocity(1), scaled_right_linear_velocity(2), 0,
        -scaled_right_linear_velocity(0), -scaled_right_linear_velocity(1), scaled_right_linear_velocity(0), 0;

    MatrixXd F = MatrixXd::Zero(6, 6);
    F.block<3, 3>(0, 0) = Matrix3d::Identity();
    F.block<3, 3>(0, 3) =
        -0.5 * dt *
        (preintegration_gamma.toRotationMatrix() * R_v0 +
         preintegration_gamma.toRotationMatrix() * R_v1 * orientation_increment.toRotationMatrix().transpose());
    F.block<3, 3>(3, 3) = orientation_increment.toRotationMatrix().transpose();

    Eigen::Matrix3d right_orientation_jacobian;
    Sophus::rightJacobianSO3(mid_angular_velocity * dt, right_orientation_jacobian);

    MatrixXd V = MatrixXd::Zero(6, 12);
    V.block<3, 3>(0, 0) = 0.5 * dt * preintegration_gamma.toRotationMatrix() * sv;
    V.block<3, 3>(0, 3) = -0.25 * dt * dt * preintegration_gamma.toRotationMatrix() * R_v1 * right_orientation_jacobian;
    V.block<3, 3>(0, 6) = 0.5 * dt * preintegration_gamma.toRotationMatrix() * sv;
    V.block<3, 3>(0, 9) = -0.25 * dt * dt * preintegration_gamma.toRotationMatrix() * R_v1 * right_orientation_jacobian;
    V.block<3, 3>(3, 3) = 0.5 * right_orientation_jacobian * linearized_sw * dt;
    V.block<3, 3>(3, 9) = 0.5 * right_orientation_jacobian * linearized_sw * dt;

    Eigen::Matrix3d unit_x_diagonal_matrix = Eigen::Vector3d(1, 0, 0).asDiagonal();
    Eigen::Matrix3d unit_y_diagonal_matrix = Eigen::Vector3d(0, 1, 0).asDiagonal();

    // TODO: change according to a number of features tracked
    Eigen::Matrix<double, 12, 12> W = VO_FAILURE_WEIGHT * Eigen::Matrix<double, 12, 12>::Identity();

    jacobian.block<3, 1>(0, 0) =
        jacobian.block<3, 1>(0, 0).eval() +
        0.5 *
            (preintegration_gamma.toRotationMatrix() * unit_x_diagonal_matrix * latest_linear_velocity +
             preintegration_gamma.toRotationMatrix() * unit_x_diagonal_matrix * input_linear_velocity) *
            dt;
    jacobian.block<3, 1>(0, 1) =
        jacobian.block<3, 1>(0, 1).eval() +
        0.5 *
            (preintegration_gamma.toRotationMatrix() * unit_y_diagonal_matrix * latest_linear_velocity +
             preintegration_gamma.toRotationMatrix() * unit_y_diagonal_matrix * input_linear_velocity) *
            dt;
    Eigen::Vector3d dr_dsw_last = jacobian.block<3, 1>(3, 2);
    jacobian.block<3, 1>(3, 2) =
        jacobian.block<3, 1>(3, 2).eval() +
        right_orientation_jacobian * 0.5 * (latest_angular_velocity + input_angular_velocity) * dt;
    jacobian.block<3, 1>(0, 2) =
        jacobian.block<3, 1>(0, 2).eval() +
        0.5 *
            (preintegration_gamma.toRotationMatrix() * Utility::skewSymmetric(dr_dsw_last) * sv *
                 latest_linear_velocity +
             preintegration_gamma.toRotationMatrix() * Utility::skewSymmetric(jacobian.block<3, 1>(3, 2)) * sv *
                 input_linear_velocity) *
            dt;

    covariance = F * covariance * F.transpose() + V * W * noise * W.transpose() * V.transpose();
  }

  void propagate(double input_dt, const Eigen::Vector3d &input_linear_velocity_data,
                 const Eigen::Vector3d &input_angular_velocity_data) {
    dt = input_dt;
    input_linear_velocity = input_linear_velocity_data;
    input_angular_velocity = input_angular_velocity_data;

    midPointIntegration();

    preintegration_gamma.normalize();
    sum_dt += dt;
    latest_linear_velocity = input_linear_velocity;
    latest_angular_velocity = input_angular_velocity;
  }

  Eigen::Matrix<double, 6, 1> evaluate(const Eigen::Vector3d &Pi, const Eigen::Quaterniond &Qi,
                                       const Eigen::Quaterniond &qio, const Eigen::Vector3d &tio, const double sx,
                                       const double sy, const double sw, const Eigen::Vector3d &Pj,
                                       const Eigen::Quaterniond &Qj, const double td) {
    Eigen::Matrix<double, 6, 1> residuals;

    Eigen::Vector3d dp_dsx = jacobian.block<3, 1>(0, 0);
    Eigen::Vector3d dp_dsy = jacobian.block<3, 1>(0, 1);
    Eigen::Vector3d dp_dsw = jacobian.block<3, 1>(0, 2);
    Eigen::Vector3d dq_dsw = jacobian.block<3, 1>(3, 2);

    double dsx = sx - linearized_sx;
    double dsy = sy - linearized_sy;
    double dsw = sw - linearized_sw;
    Eigen::Matrix3d sv = Eigen::Vector3d(sx, sy, 1).asDiagonal();
    Eigen::Matrix3d Ri = Qi.toRotationMatrix();
    Eigen::Matrix3d Rj = Qj.toRotationMatrix();
    Eigen::Matrix3d Rio = qio.toRotationMatrix();

    corrected_preintegration_alpha = preintegration_alpha + dp_dsx * dsx + dp_dsy * dsy + dp_dsw * dsw;
    corrected_preintegration_gamma =
        (Sophus::SO3d(preintegration_gamma) * Sophus::SO3d::exp(dq_dsw * dsw)).unit_quaternion();
    double dtd = td - linearized_td;

    Eigen::Quaterniond delta_q_time =
        (Sophus::SO3d::exp(sw * linearized_angular_velocity * dtd) * Sophus::SO3d(corrected_preintegration_gamma) *
         Sophus::SO3d::exp(-sw * input_angular_velocity * dtd))
            .unit_quaternion();
    Eigen::Vector3d delta_p_time = Sophus::SO3d::exp(sw * linearized_angular_velocity * dtd).matrix() *
                                   (sv * linearized_linear_velocity * dtd + corrected_preintegration_alpha -
                                    corrected_preintegration_gamma * sv * input_linear_velocity * dtd);

    residuals.block<3, 1>(O_P, 0) = (Ri * Rio).transpose() * (Rj * tio + Pj - Ri * tio - Pi) - delta_p_time;
    residuals.block<3, 1>(O_R, 0) = Sophus::SO3d(delta_q_time.inverse() * (Qi * qio).inverse() * Qj * qio).log();
    return residuals;
  }

  double dt;
  Eigen::Vector3d latest_linear_velocity, latest_angular_velocity;
  Eigen::Vector3d input_linear_velocity, input_angular_velocity;

  const Eigen::Vector3d linearized_linear_velocity, linearized_angular_velocity;
  double linearized_sx, linearized_sy, linearized_sw;
  double linearized_td;
  Eigen::Matrix<double, 6, 3> jacobian;
  Eigen::Matrix<double, 6, 6> covariance;
  Eigen::Matrix<double, 12, 12> noise;

  double sum_dt;
  Eigen::Vector3d preintegration_alpha;
  Eigen::Quaterniond preintegration_gamma;
  Eigen::Vector3d corrected_preintegration_alpha;
  Eigen::Quaterniond corrected_preintegration_gamma;

  std::vector<double> dt_buffer;
  std::vector<Eigen::Vector3d> linear_velocity_buffer;
  std::vector<Eigen::Vector3d> angular_velocity_buffer;
};