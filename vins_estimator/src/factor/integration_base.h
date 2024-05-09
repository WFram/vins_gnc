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
#include "../utility/utility.h"

#include <ceres/ceres.h>
using namespace Eigen;

class IntegrationBase {
 public:
  IntegrationBase() = delete;
  IntegrationBase(const Eigen::Vector3d &input_acc_data, const Eigen::Vector3d &input_gyro_data,
                  const Eigen::Vector3d &input_acc_bias_data, const Eigen::Vector3d &input_gyro_bias_data)
      : latest_linear_acceleration{input_acc_data},
        latest_angular_velocity{input_gyro_data},
        linearized_acc_measurement{input_acc_data},
        linearized_gyro_measurement{input_gyro_data},
        linearized_acc_bias{input_acc_bias_data},
        linearized_gyro_bias{input_gyro_bias_data},
        jacobian{Eigen::Matrix<double, 15, 15>::Identity()},
        covariance{Eigen::Matrix<double, 15, 15>::Zero()},
        sum_dt{0.0},
        preintegration_alpha{Eigen::Vector3d::Zero()},
        preintegration_gamma{Eigen::Quaterniond::Identity()},
        preintegration_beta{Eigen::Vector3d::Zero()}

  {
    // https://img-blog.csdnimg.cn/20200303000254574.png
    noise = Eigen::Matrix<double, 18, 18>::Zero();
    noise.block<3, 3>(0, 0) = (ACC_N * ACC_N) * Eigen::Matrix3d::Identity();
    noise.block<3, 3>(3, 3) = (GYR_N * GYR_N) * Eigen::Matrix3d::Identity();
    noise.block<3, 3>(6, 6) = (ACC_N * ACC_N) * Eigen::Matrix3d::Identity();
    noise.block<3, 3>(9, 9) = (GYR_N * GYR_N) * Eigen::Matrix3d::Identity();
    noise.block<3, 3>(12, 12) = (ACC_W * ACC_W) * Eigen::Matrix3d::Identity();
    noise.block<3, 3>(15, 15) = (GYR_W * GYR_W) * Eigen::Matrix3d::Identity();
  }

  void push_back(double dt, const Eigen::Vector3d &acc, const Eigen::Vector3d &gyr) {
    dt_buffer.push_back(dt);
    acc_buffer.push_back(acc);
    gyro_buffer.push_back(gyr);
    propagate(dt, acc, gyr);
  }

  void repropagate(const Eigen::Vector3d &input_acc_bias, const Eigen::Vector3d &input_gyro_bias) {
    ROS_INFO("Repropagate IMU measurements");
    sum_dt = 0.0;
    latest_linear_acceleration = linearized_acc_measurement;
    latest_angular_velocity = linearized_gyro_measurement;
    preintegration_alpha.setZero();
    preintegration_gamma.setIdentity();
    preintegration_beta.setZero();
    linearized_acc_bias = input_acc_bias;
    linearized_gyro_bias = input_gyro_bias;
    jacobian.setIdentity();
    covariance.setZero();
    for (int i = 0; i < static_cast<int>(dt_buffer.size()); i++) {
      propagate(dt_buffer[i], acc_buffer[i], gyro_buffer[i]);
    }
  }

  void midPointIntegration() {
    Vector3d left_linear_acceleration = preintegration_gamma * (latest_linear_acceleration - linearized_acc_bias);
    Vector3d mid_angular_velocity = 0.5 * (latest_angular_velocity + input_angular_velocity) - linearized_gyro_bias;
    preintegration_gamma =
        preintegration_gamma * Quaterniond(1, mid_angular_velocity(0) * dt / 2, mid_angular_velocity(1) * dt / 2,
                                           mid_angular_velocity(2) * dt / 2);
    Vector3d right_linear_acceleration = preintegration_gamma * (input_linear_acceleration - linearized_acc_bias);
    Vector3d mid_linear_acceleration = 0.5 * (left_linear_acceleration + right_linear_acceleration);
    preintegration_alpha = preintegration_alpha + preintegration_beta * dt + 0.5 * mid_linear_acceleration * dt * dt;
    preintegration_beta = preintegration_beta + mid_linear_acceleration * dt;

    Vector3d unbiased_left_linear_acceleration = latest_linear_acceleration - linearized_acc_bias;
    Vector3d unbiased_right_linear_acceleration = input_linear_acceleration - linearized_acc_bias;
    Matrix3d R_w, R_a0, R_a1;

    R_w << 0, -mid_angular_velocity(2), mid_angular_velocity(1), mid_angular_velocity(2), 0, -mid_angular_velocity(0),
        -mid_angular_velocity(1), mid_angular_velocity(0), 0;
    R_a0 << 0, -unbiased_left_linear_acceleration(2), unbiased_left_linear_acceleration(1),
        unbiased_left_linear_acceleration(2), 0, -unbiased_left_linear_acceleration(0),
        -unbiased_left_linear_acceleration(1), unbiased_left_linear_acceleration(0), 0;
    R_a1 << 0, -unbiased_right_linear_acceleration(2), unbiased_right_linear_acceleration(1),
        unbiased_right_linear_acceleration(2), 0, -unbiased_right_linear_acceleration(0),
        -unbiased_right_linear_acceleration(1), unbiased_right_linear_acceleration(0), 0;

    MatrixXd F = MatrixXd::Zero(15, 15);
    F.block<3, 3>(0, 0) = Matrix3d::Identity();
    F.block<3, 3>(0, 3) =
        -0.25 * preintegration_gamma.toRotationMatrix() * R_a0 * dt * dt +
        -0.25 * preintegration_gamma.toRotationMatrix() * R_a1 * (Matrix3d::Identity() - R_w * dt) * dt * dt;
    F.block<3, 3>(0, 6) = MatrixXd::Identity(3, 3) * dt;
    F.block<3, 3>(0, 9) =
        -0.25 * (preintegration_gamma.toRotationMatrix() + preintegration_gamma.toRotationMatrix()) * dt * dt;
    F.block<3, 3>(0, 12) = -0.25 * preintegration_gamma.toRotationMatrix() * R_a1 * dt * dt * -dt;
    F.block<3, 3>(3, 3) = Matrix3d::Identity() - R_w * dt;
    F.block<3, 3>(3, 12) = -1.0 * MatrixXd::Identity(3, 3) * dt;
    F.block<3, 3>(6, 3) =
        -0.5 * preintegration_gamma.toRotationMatrix() * R_a0 * dt +
        -0.5 * preintegration_gamma.toRotationMatrix() * R_a1 * (Matrix3d::Identity() - R_w * dt) * dt;
    F.block<3, 3>(6, 6) = Matrix3d::Identity();
    F.block<3, 3>(6, 9) =
        -0.5 * (preintegration_gamma.toRotationMatrix() + preintegration_gamma.toRotationMatrix()) * dt;
    F.block<3, 3>(6, 12) = -0.5 * preintegration_gamma.toRotationMatrix() * R_a1 * dt * -dt;
    F.block<3, 3>(9, 9) = Matrix3d::Identity();
    F.block<3, 3>(12, 12) = Matrix3d::Identity();

    MatrixXd V = MatrixXd::Zero(15, 18);
    V.block<3, 3>(0, 0) = 0.25 * preintegration_gamma.toRotationMatrix() * dt * dt;
    V.block<3, 3>(0, 3) = 0.25 * -preintegration_gamma.toRotationMatrix() * R_a1 * dt * dt * 0.5 * dt;
    V.block<3, 3>(0, 6) = 0.25 * preintegration_gamma.toRotationMatrix() * dt * dt;
    V.block<3, 3>(0, 9) = V.block<3, 3>(0, 3);
    V.block<3, 3>(3, 3) = 0.5 * MatrixXd::Identity(3, 3) * dt;
    V.block<3, 3>(3, 9) = 0.5 * MatrixXd::Identity(3, 3) * dt;
    V.block<3, 3>(6, 0) = 0.5 * preintegration_gamma.toRotationMatrix() * dt;
    V.block<3, 3>(6, 3) = 0.5 * -preintegration_gamma.toRotationMatrix() * R_a1 * dt * 0.5 * dt;
    V.block<3, 3>(6, 6) = 0.5 * preintegration_gamma.toRotationMatrix() * dt;
    V.block<3, 3>(6, 9) = V.block<3, 3>(6, 3);
    V.block<3, 3>(9, 12) = MatrixXd::Identity(3, 3) * dt;
    V.block<3, 3>(12, 15) = MatrixXd::Identity(3, 3) * dt;

    jacobian = F * jacobian;
    covariance = F * covariance * F.transpose() + V * noise * V.transpose();
  }

  void propagate(double input_dt, const Eigen::Vector3d &input_acc_data, const Eigen::Vector3d &input_gyro_data) {
    dt = input_dt;
    input_linear_acceleration = input_acc_data;
    input_angular_velocity = input_gyro_data;

    midPointIntegration();

    preintegration_gamma.normalize();
    sum_dt += dt;
    latest_linear_acceleration = input_linear_acceleration;
    latest_angular_velocity = input_angular_velocity;
  }

  Eigen::Matrix<double, 15, 1> evaluate(const Eigen::Vector3d &Pi, const Eigen::Quaterniond &Qi,
                                        const Eigen::Vector3d &Vi, const Eigen::Vector3d &Bai,
                                        const Eigen::Vector3d &Bgi, const Eigen::Vector3d &Pj,
                                        const Eigen::Quaterniond &Qj, const Eigen::Vector3d &Vj,
                                        const Eigen::Vector3d &Baj, const Eigen::Vector3d &Bgj) {
    Eigen::Matrix<double, 15, 1> residuals;

    Eigen::Matrix3d dp_dba = jacobian.block<3, 3>(O_P, O_BA);
    Eigen::Matrix3d dp_dbg = jacobian.block<3, 3>(O_P, O_BG);

    Eigen::Matrix3d dq_dbg = jacobian.block<3, 3>(O_R, O_BG);

    Eigen::Matrix3d dv_dba = jacobian.block<3, 3>(O_V, O_BA);
    Eigen::Matrix3d dv_dbg = jacobian.block<3, 3>(O_V, O_BG);

    Eigen::Vector3d dba = Bai - linearized_acc_bias;
    Eigen::Vector3d dbg = Bgi - linearized_gyro_bias;

    // https://blog.csdn.net/iwanderu/article/details/104623177
    Eigen::Quaterniond corrected_preintegration_gamma = preintegration_gamma * Utility::deltaQ(dq_dbg * dbg);
    Eigen::Vector3d corrected_preintegration_beta = preintegration_beta + dv_dba * dba + dv_dbg * dbg;
    Eigen::Vector3d corrected_preintegration_alpha = preintegration_alpha + dp_dba * dba + dp_dbg * dbg;

    // https://blog.csdn.net/iwanderu/article/details/104729332?spm=1001.2101.3001.6650.1&utm_medium=distribute.pc_relevant.none-task-blog-2%7Edefault%7ECTRLIST%7ERate-1-104729332-blog-102782554.pc_relevant_multi_platform_whitelistv4&depth_1-utm_source=distribute.pc_relevant.none-task-blog-2%7Edefault%7ECTRLIST%7ERate-1-104729332-blog-102782554.pc_relevant_multi_platform_whitelistv4&utm_relevant_index=2
    residuals.block<3, 1>(O_P, 0) =
        Qi.inverse() * (0.5 * G * sum_dt * sum_dt + Pj - Pi - Vi * sum_dt) - corrected_preintegration_alpha;
    residuals.block<3, 1>(O_R, 0) = 2 * (corrected_preintegration_gamma.inverse() * (Qi.inverse() * Qj)).vec();
    residuals.block<3, 1>(O_V, 0) = Qi.inverse() * (G * sum_dt + Vj - Vi) - corrected_preintegration_beta;
    residuals.block<3, 1>(O_BA, 0) = Baj - Bai;
    residuals.block<3, 1>(O_BG, 0) = Bgj - Bgi;
    return residuals;
  }

  double dt;
  Eigen::Vector3d latest_linear_acceleration, latest_angular_velocity;
  Eigen::Vector3d input_linear_acceleration, input_angular_velocity;

  const Eigen::Vector3d linearized_acc_measurement, linearized_gyro_measurement;
  Eigen::Vector3d linearized_acc_bias, linearized_gyro_bias;

  Eigen::Matrix<double, 15, 15> jacobian, covariance;
  Eigen::Matrix<double, 18, 18> noise;

  double sum_dt;
  Eigen::Vector3d preintegration_alpha;
  Eigen::Quaterniond preintegration_gamma;
  Eigen::Vector3d preintegration_beta;

  std::vector<double> dt_buffer;
  std::vector<Eigen::Vector3d> acc_buffer;
  std::vector<Eigen::Vector3d> gyro_buffer;
};