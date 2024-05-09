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

#include "initial_alignment.h"

void solveGyroscopeBias(map<double, ImageFrame> &accumulated_image_frames, Vector3d *gyro_bias) {
  ROS_INFO("Solving gyro bias");
  Matrix3d hessian;
  Vector3d b;
  Vector3d delta_bg;
  hessian.setZero();
  b.setZero();
  map<double, ImageFrame>::iterator frame_i;
  map<double, ImageFrame>::iterator frame_j;
  for (frame_i = accumulated_image_frames.begin(); next(frame_i) != accumulated_image_frames.end(); frame_i++) {
    frame_j = next(frame_i);
    MatrixXd jacobian(3, 3);
    jacobian.setZero();
    VectorXd residual(3);
    residual.setZero();
    Eigen::Quaterniond q_ij(frame_i->second.rotation.transpose() * frame_j->second.rotation);
    jacobian = frame_j->second.imu_preintegration->jacobian.template block<3, 3>(O_R, O_BG);
    residual = 2 * (frame_j->second.imu_preintegration->preintegration_gamma.inverse() * q_ij).vec();
    hessian += jacobian.transpose() * jacobian;
    b += jacobian.transpose() * residual;
  }
  delta_bg = hessian.ldlt().solve(b);
  ROS_INFO_STREAM("Gyroscope bias initial estimation " << delta_bg.transpose());

  for (int i = 0; i <= WINDOW_SIZE; i++) {
    gyro_bias[i] += delta_bg;
  }

  for (frame_i = accumulated_image_frames.begin(); next(frame_i) != accumulated_image_frames.end(); frame_i++) {
    frame_j = next(frame_i);
    frame_j->second.imu_preintegration->repropagate(Vector3d::Zero(), gyro_bias[0]);
  }
}

MatrixXd TangentBasis(Vector3d &gravity_vector) {
  Vector3d b1, b2;
  Vector3d normalized_gravity_vector = gravity_vector.normalized();
  Vector3d axis(0, 0, 1);
  if (normalized_gravity_vector == axis) axis << 1, 0, 0;
  b1 = (axis - normalized_gravity_vector * (normalized_gravity_vector.transpose() * axis)).normalized();
  b2 = normalized_gravity_vector.cross(b1);
  MatrixXd tangent_basis(3, 2);
  tangent_basis.block<3, 1>(0, 0) = b1;
  tangent_basis.block<3, 1>(0, 1) = b2;
  return tangent_basis;
}

void RefineGravity(map<double, ImageFrame> &accumulated_image_frames, Vector3d &gravity_vector, VectorXd &state) {
  Vector3d gravity_vector_estimate = gravity_vector.normalized() * G.norm();
  int all_frame_count = static_cast<int>(accumulated_image_frames.size());
  int n_state = all_frame_count * 3 + 2 + 1;

  MatrixXd hessian{n_state, n_state};
  hessian.setZero();
  VectorXd b{n_state};
  b.setZero();

  map<double, ImageFrame>::iterator frame_i;
  map<double, ImageFrame>::iterator frame_j;
  for (int k = 0; k < 4; k++) {
    MatrixXd tangent_gravity_basis(3, 2);
    tangent_gravity_basis = TangentBasis(gravity_vector_estimate);
    int i = 0;
    for (frame_i = accumulated_image_frames.begin(); next(frame_i) != accumulated_image_frames.end(); frame_i++, i++) {
      frame_j = next(frame_i);

      MatrixXd jacobian(6, 9);
      jacobian.setZero();
      VectorXd residual(6);
      residual.setZero();

      double dt = frame_j->second.imu_preintegration->sum_dt;

      jacobian.block<3, 3>(0, 0) = -dt * Matrix3d::Identity();
      jacobian.block<3, 2>(0, 6) =
          frame_i->second.rotation.transpose() * dt * dt / 2 * Matrix3d::Identity() * tangent_gravity_basis;
      jacobian.block<3, 1>(0, 8) =
          frame_i->second.rotation.transpose() * (frame_j->second.translation - frame_i->second.translation) / 100.0;
      residual.block<3, 1>(0, 0) = frame_j->second.imu_preintegration->preintegration_alpha +
                                   frame_i->second.rotation.transpose() * frame_j->second.rotation * TIC[0] - TIC[0] -
                                   frame_i->second.rotation.transpose() * dt * dt / 2 * gravity_vector_estimate;

      jacobian.block<3, 3>(3, 0) = -Matrix3d::Identity();
      jacobian.block<3, 3>(3, 3) = frame_i->second.rotation.transpose() * frame_j->second.rotation;
      jacobian.block<3, 2>(3, 6) =
          frame_i->second.rotation.transpose() * dt * Matrix3d::Identity() * tangent_gravity_basis;
      residual.block<3, 1>(3, 0) =
          frame_j->second.imu_preintegration->preintegration_beta -
          frame_i->second.rotation.transpose() * dt * Matrix3d::Identity() * gravity_vector_estimate;

      Matrix<double, 6, 6> information = Matrix<double, 6, 6>::Zero();
      information.setIdentity();

      MatrixXd whitened_jacobian = jacobian.transpose() * information * jacobian;
      VectorXd whitened_residual = jacobian.transpose() * information * residual;

      hessian.block<6, 6>(i * 3, i * 3) += whitened_jacobian.topLeftCorner<6, 6>();
      b.segment<6>(i * 3) += whitened_residual.head<6>();

      hessian.bottomRightCorner<3, 3>() += whitened_jacobian.bottomRightCorner<3, 3>();
      b.tail<3>() += whitened_residual.tail<3>();

      hessian.block<6, 3>(i * 3, n_state - 3) += whitened_jacobian.topRightCorner<6, 3>();
      hessian.block<3, 6>(n_state - 3, i * 3) += whitened_jacobian.bottomLeftCorner<3, 6>();
    }
    hessian = hessian * 1000.0;
    b = b * 1000.0;
    state = hessian.ldlt().solve(b);
    VectorXd dg = state.segment<2>(n_state - 3);
    gravity_vector_estimate = (gravity_vector_estimate + tangent_gravity_basis * dg).normalized() * G.norm();
  }
  gravity_vector = gravity_vector_estimate;
}

bool LinearIMUAlignment(map<double, ImageFrame> &accumulated_image_frames, Vector3d &gravity_vector, VectorXd &state) {
  int number_of_frames = static_cast<int>(accumulated_image_frames.size());
  int state_dimensions = number_of_frames * 3 + 3 + 1;

  MatrixXd hessian{state_dimensions, state_dimensions};
  hessian.setZero();
  VectorXd b{state_dimensions};
  b.setZero();

  map<double, ImageFrame>::iterator frame_i;
  map<double, ImageFrame>::iterator frame_j;
  int i = 0;
  for (frame_i = accumulated_image_frames.begin(); next(frame_i) != accumulated_image_frames.end(); frame_i++, i++) {
    frame_j = next(frame_i);

    MatrixXd jacobian(6, 10);
    jacobian.setZero();
    VectorXd residual(6);
    residual.setZero();

    double dt = frame_j->second.imu_preintegration->sum_dt;

    jacobian.block<3, 3>(0, 0) = -dt * Matrix3d::Identity();
    jacobian.block<3, 3>(0, 6) = frame_i->second.rotation.transpose() * dt * dt / 2 * Matrix3d::Identity();
    jacobian.block<3, 1>(0, 9) =
        frame_i->second.rotation.transpose() * (frame_j->second.translation - frame_i->second.translation) / 100.0;
    residual.block<3, 1>(0, 0) = frame_j->second.imu_preintegration->preintegration_alpha +
                                 frame_i->second.rotation.transpose() * frame_j->second.rotation * TIC[0] - TIC[0];
    jacobian.block<3, 3>(3, 0) = -Matrix3d::Identity();
    jacobian.block<3, 3>(3, 3) = frame_i->second.rotation.transpose() * frame_j->second.rotation;
    jacobian.block<3, 3>(3, 6) = frame_i->second.rotation.transpose() * dt * Matrix3d::Identity();
    residual.block<3, 1>(3, 0) = frame_j->second.imu_preintegration->preintegration_beta;

    Matrix<double, 6, 6> information = Matrix<double, 6, 6>::Zero();
    information.setIdentity();

    MatrixXd whitened_jacobian = jacobian.transpose() * information * jacobian;
    VectorXd whitened_residual = jacobian.transpose() * information * residual;

    hessian.block<6, 6>(i * 3, i * 3) += whitened_jacobian.topLeftCorner<6, 6>();
    b.segment<6>(i * 3) += whitened_residual.head<6>();

    hessian.bottomRightCorner<4, 4>() += whitened_jacobian.bottomRightCorner<4, 4>();
    b.tail<4>() += whitened_residual.tail<4>();

    hessian.block<6, 4>(i * 3, state_dimensions - 4) += whitened_jacobian.topRightCorner<6, 4>();
    hessian.block<4, 6>(state_dimensions - 4, i * 3) += whitened_jacobian.bottomLeftCorner<4, 6>();
  }
  hessian = hessian * 1000.0;
  b = b * 1000.0;
  state = hessian.ldlt().solve(b);
  double scale = state(state_dimensions - 1) / 100.0;
  ROS_DEBUG("Estimated scale: %f", scale);
  gravity_vector = state.segment<3>(state_dimensions - 4);
  ROS_DEBUG_STREAM("Estimated gravity vector: " << gravity_vector.norm() << " " << gravity_vector.transpose());
  if (fabs(gravity_vector.norm() - G.norm()) > 0.5 || scale < 0) {
    return false;
  }

  RefineGravity(accumulated_image_frames, gravity_vector, state);
  scale = (state.tail<1>())(0) / 100.0;
  (state.tail<1>())(0) = scale;
  ROS_DEBUG_STREAM("Refined gravity vector: " << gravity_vector.norm() << " " << gravity_vector.transpose());
  return scale >= 0.0;
}

void RefineGravityWithWheel(map<double, ImageFrame> &accumulated_image_frames, Vector3d &gravity_vector,
                            VectorXd &state) {
  ROS_INFO("Refine gravity with wheel odometry");
  Vector3d gravity_vector_estimate = gravity_vector.normalized() * G.norm();
  int number_of_frames = static_cast<int>(accumulated_image_frames.size());
  int state_dimensions = number_of_frames * 3 + 2 + 1;

  MatrixXd hessian{state_dimensions, state_dimensions};
  hessian.setZero();
  VectorXd b{state_dimensions};
  b.setZero();

  map<double, ImageFrame>::iterator frame_i;
  map<double, ImageFrame>::iterator frame_j;
  for (int k = 0; k < 4; k++) {
    MatrixXd tangent_gravity_basis(3, 2);
    tangent_gravity_basis = TangentBasis(gravity_vector_estimate);
    int i = 0;
    for (frame_i = accumulated_image_frames.begin(); next(frame_i) != accumulated_image_frames.end(); frame_i++, i++) {
      frame_j = next(frame_i);

      MatrixXd jacobian(9, 9);
      jacobian.setZero();
      VectorXd residual(9);
      residual.setZero();

      double dt = frame_j->second.imu_preintegration->sum_dt;

      jacobian.block<3, 3>(0, 0) = -dt * Matrix3d::Identity();
      jacobian.block<3, 2>(0, 6) =
          frame_i->second.rotation.transpose() * dt * dt / 2 * Matrix3d::Identity() * tangent_gravity_basis;
      jacobian.block<3, 1>(0, 8) =
          frame_i->second.rotation.transpose() * (frame_j->second.translation - frame_i->second.translation) / 100.0;
      residual.block<3, 1>(0, 0) = frame_j->second.imu_preintegration->preintegration_alpha +
                                   frame_i->second.rotation.transpose() * frame_j->second.rotation * TIC[0] - TIC[0] -
                                   frame_i->second.rotation.transpose() * dt * dt / 2 * gravity_vector_estimate;

      jacobian.block<3, 3>(3, 0) = -Matrix3d::Identity();
      jacobian.block<3, 3>(3, 3) = frame_i->second.rotation.transpose() * frame_j->second.rotation;
      jacobian.block<3, 2>(3, 6) =
          frame_i->second.rotation.transpose() * dt * Matrix3d::Identity() * tangent_gravity_basis;
      residual.block<3, 1>(3, 0) =
          frame_j->second.imu_preintegration->preintegration_beta -
          frame_i->second.rotation.transpose() * dt * Matrix3d::Identity() * gravity_vector_estimate;

      jacobian.block<3, 1>(6, 8) = (frame_i->second.rotation * RIO).transpose() *
                                   (frame_j->second.translation - frame_i->second.translation) / 100;
      residual.block<3, 1>(6, 0) =
          frame_j->second.pre_integration_wheel->preintegration_alpha -
          RIO.transpose() * frame_i->second.rotation.transpose() * frame_j->second.rotation * TIO +
          (frame_i->second.rotation * RIO).transpose() * frame_j->second.rotation * TIC[0] -
          RIO.transpose() * (TIC[0] - TIO);

      Matrix<double, 9, 9> information = Matrix<double, 9, 9>::Zero();
      information.setIdentity();

      MatrixXd whitened_jacobian = jacobian.transpose() * information * jacobian;
      VectorXd whitened_residual = jacobian.transpose() * information * residual;

      hessian.block<6, 6>(i * 3, i * 3) += whitened_jacobian.topLeftCorner<6, 6>();
      b.segment<6>(i * 3) += whitened_residual.head<6>();

      hessian.bottomRightCorner<3, 3>() += whitened_jacobian.bottomRightCorner<3, 3>();
      b.tail<3>() += whitened_residual.tail<3>();

      hessian.block<6, 3>(i * 3, state_dimensions - 3) += whitened_jacobian.topRightCorner<6, 3>();
      hessian.block<3, 6>(state_dimensions - 3, i * 3) += whitened_jacobian.bottomLeftCorner<3, 6>();
    }
    hessian = hessian * 1000.0;
    b = b * 1000.0;
    state = hessian.ldlt().solve(b);
    VectorXd dg = state.segment<2>(state_dimensions - 3);
    gravity_vector_estimate = (gravity_vector_estimate + tangent_gravity_basis * dg).normalized() * G.norm();
    // TODO: get scale here
  }
  gravity_vector = gravity_vector_estimate;
}

bool LinearWheelAlignment(map<double, ImageFrame> &accumulated_image_frames, Vector3d &gravity_vector,
                          VectorXd &state) {
  ROS_INFO("Perform linear alignment with wheel odometry");
  int number_of_frames = static_cast<int>(accumulated_image_frames.size());
  int state_dimensions = number_of_frames * 3 + 3 + 1;

  MatrixXd hessian{state_dimensions, state_dimensions};
  hessian.setZero();
  VectorXd b{state_dimensions};
  b.setZero();

  map<double, ImageFrame>::iterator frame_i;
  map<double, ImageFrame>::iterator frame_j;
  int i = 0;
  for (frame_i = accumulated_image_frames.begin(); next(frame_i) != accumulated_image_frames.end(); frame_i++, i++) {
    frame_j = next(frame_i);

    MatrixXd jacobian(9, 10);
    jacobian.setZero();
    VectorXd residual(9);
    residual.setZero();

    double dt = frame_j->second.imu_preintegration->sum_dt;

    jacobian.block<3, 3>(0, 0) = -dt * Matrix3d::Identity();
    jacobian.block<3, 3>(0, 6) = frame_i->second.rotation.transpose() * dt * dt / 2 * Matrix3d::Identity();
    jacobian.block<3, 1>(0, 9) =
        frame_i->second.rotation.transpose() * (frame_j->second.translation - frame_i->second.translation) / 100.0;
    residual.block<3, 1>(0, 0) = frame_j->second.imu_preintegration->preintegration_alpha +
                                 frame_i->second.rotation.transpose() * frame_j->second.rotation * TIC[0] - TIC[0];
    jacobian.block<3, 3>(3, 0) = -Matrix3d::Identity();
    jacobian.block<3, 3>(3, 3) = frame_i->second.rotation.transpose() * frame_j->second.rotation;
    jacobian.block<3, 3>(3, 6) = frame_i->second.rotation.transpose() * dt * Matrix3d::Identity();
    residual.block<3, 1>(3, 0) = frame_j->second.imu_preintegration->preintegration_beta;

    jacobian.block<3, 1>(6, 9) = (frame_i->second.rotation * RIO).transpose() *
                                 (frame_j->second.translation - frame_i->second.translation) / 100;
    residual.block<3, 1>(6, 0) =
        frame_j->second.pre_integration_wheel->preintegration_alpha -
        RIO.transpose() * frame_i->second.rotation.transpose() * frame_j->second.rotation * TIO +
        (frame_i->second.rotation * RIO).transpose() * frame_j->second.rotation * TIC[0] -
        RIO.transpose() * (TIC[0] - TIO);

    Matrix<double, 9, 9> information = Matrix<double, 9, 9>::Zero();
    information.setIdentity();

    MatrixXd whitened_jacobian = jacobian.transpose() * information * jacobian;
    VectorXd whitened_residual = jacobian.transpose() * information * residual;

    hessian.block<6, 6>(i * 3, i * 3) += whitened_jacobian.topLeftCorner<6, 6>();
    b.segment<6>(i * 3) += whitened_residual.head<6>();

    hessian.bottomRightCorner<4, 4>() += whitened_jacobian.bottomRightCorner<4, 4>();
    b.tail<4>() += whitened_residual.tail<4>();

    hessian.block<6, 4>(i * 3, state_dimensions - 4) += whitened_jacobian.topRightCorner<6, 4>();
    hessian.block<4, 6>(state_dimensions - 4, i * 3) += whitened_jacobian.bottomLeftCorner<4, 6>();
  }
  hessian = hessian * 1000.0;
  b = b * 1000.0;
  state = hessian.ldlt().solve(b);
  double scale = state(state_dimensions - 1) / 100.0;
  ROS_INFO("Estimated scale: %f", scale);
  gravity_vector << 0.0, -9.805, 0.0;
  ROS_INFO_STREAM("Estimated gravity vector: " << gravity_vector.norm() << " " << gravity_vector.transpose());

  scale = (state.tail<1>())(0) / 100.0;
  (state.tail<1>())(0) = scale;
  return scale >= 0.0;
}

bool LinearIMUWheelAlignment(map<double, ImageFrame> &accumulated_image_frames, Vector3d &gravity_vector,
                             VectorXd &state) {
  ROS_INFO("Perform linear alignment with IMU and wheel odometry");
  int number_of_frames = static_cast<int>(accumulated_image_frames.size());
  int state_dimensions = number_of_frames * 3 + 3 + 1;

  MatrixXd hessian{state_dimensions, state_dimensions};
  hessian.setZero();
  VectorXd b{state_dimensions};
  b.setZero();

  map<double, ImageFrame>::iterator frame_i;
  map<double, ImageFrame>::iterator frame_j;
  int i = 0;
  for (frame_i = accumulated_image_frames.begin(); next(frame_i) != accumulated_image_frames.end(); frame_i++, i++) {
    frame_j = next(frame_i);

    MatrixXd jacobian(9, 10);
    jacobian.setZero();
    VectorXd residual(9);
    residual.setZero();

    double dt = frame_j->second.imu_preintegration->sum_dt;

    jacobian.block<3, 3>(0, 0) = -dt * Matrix3d::Identity();
    jacobian.block<3, 3>(0, 6) = frame_i->second.rotation.transpose() * dt * dt / 2 * Matrix3d::Identity();
    jacobian.block<3, 1>(0, 9) =
        frame_i->second.rotation.transpose() * (frame_j->second.translation - frame_i->second.translation) / 100.0;
    residual.block<3, 1>(0, 0) = frame_j->second.imu_preintegration->preintegration_alpha +
                                 frame_i->second.rotation.transpose() * frame_j->second.rotation * TIC[0] - TIC[0];
    jacobian.block<3, 3>(3, 0) = -Matrix3d::Identity();
    jacobian.block<3, 3>(3, 3) = frame_i->second.rotation.transpose() * frame_j->second.rotation;
    jacobian.block<3, 3>(3, 6) = frame_i->second.rotation.transpose() * dt * Matrix3d::Identity();
    residual.block<3, 1>(3, 0) = frame_j->second.imu_preintegration->preintegration_beta;

    jacobian.block<3, 1>(6, 9) = (frame_i->second.rotation * RIO).transpose() *
                                 (frame_j->second.translation - frame_i->second.translation) / 100;
    residual.block<3, 1>(6, 0) =
        frame_j->second.pre_integration_wheel->preintegration_alpha -
        RIO.transpose() * frame_i->second.rotation.transpose() * frame_j->second.rotation * TIO +
        (frame_i->second.rotation * RIO).transpose() * frame_j->second.rotation * TIC[0] -
        RIO.transpose() * (TIC[0] - TIO);

    Matrix<double, 9, 9> information = Matrix<double, 9, 9>::Zero();
    information.setIdentity();

    MatrixXd whitened_jacobian = jacobian.transpose() * information * jacobian;
    VectorXd whitened_residual = jacobian.transpose() * information * residual;

    hessian.block<6, 6>(i * 3, i * 3) += whitened_jacobian.topLeftCorner<6, 6>();
    b.segment<6>(i * 3) += whitened_residual.head<6>();

    hessian.bottomRightCorner<4, 4>() += whitened_jacobian.bottomRightCorner<4, 4>();
    b.tail<4>() += whitened_residual.tail<4>();

    hessian.block<6, 4>(i * 3, state_dimensions - 4) += whitened_jacobian.topRightCorner<6, 4>();
    hessian.block<4, 6>(state_dimensions - 4, i * 3) += whitened_jacobian.bottomLeftCorner<4, 6>();
  }
  hessian = hessian * 1000.0;
  b = b * 1000.0;
  state = hessian.ldlt().solve(b);
  double scale = state(state_dimensions - 1) / 100.0;
  ROS_INFO("Estimated scale: %f", scale);
  gravity_vector = state.segment<3>(state_dimensions - 4);
  ROS_INFO_STREAM(" result g     " << gravity_vector.norm() << " " << gravity_vector.transpose());
  float gravity_difference = fabs(gravity_vector.norm() - G.norm());
  if (gravity_difference > 0.5 || scale < 0) {
    ROS_INFO("Gravity difference: %f. Failed", gravity_difference);
    return false;
  }

  RefineGravityWithWheel(accumulated_image_frames, gravity_vector, state);
  scale = (state.tail<1>())(0) / 100.0;
  (state.tail<1>())(0) = scale;
  ROS_INFO_STREAM("Refined gravity vector" << gravity_vector.norm() << " " << gravity_vector.transpose());
  return scale >= 0.0;
}

bool visualAlignment(map<double, ImageFrame> &accumulated_image_frames, Vector3d *gyro_bias, Vector3d &gravity_vector,
                     VectorXd &state) {
  if (USE_IMU) solveGyroscopeBias(accumulated_image_frames, gyro_bias);
  if (!USE_WHEEL)
    return LinearIMUAlignment(accumulated_image_frames, gravity_vector, state);
  else if (!USE_IMU)
    return LinearWheelAlignment(accumulated_image_frames, gravity_vector, state);
  else
    return LinearIMUWheelAlignment(accumulated_image_frames, gravity_vector, state);
}
