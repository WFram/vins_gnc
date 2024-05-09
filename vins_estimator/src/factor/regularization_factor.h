/*******************************************************
 * Copyright (C) 2019, Aerial Robotics Group, Hong Kong University of Science and Technology
 *
 * This file is part of VINS.
 *
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *******************************************************/

#pragma once
#include <ros/assert.h>
#include <eigen3/Eigen/Dense>
#include <iostream>

#include "../estimator/parameters.h"
#include "../utility/utility.h"

#include <ceres/ceres.h>

class RegularizationFactor : public ceres::SizedCostFunction<1, 1> {
 public:
  RegularizationFactor() = delete;

  explicit RegularizationFactor(double regularizer) : regularizer_(regularizer) {}

  double regularizer_;
};

class SimpleRegularizationFactor : public RegularizationFactor {
 public:
  explicit SimpleRegularizationFactor(double regularizer) : RegularizationFactor(regularizer) {}

  virtual bool Evaluate(double const *const *parameters, double *residuals, double **jacobians) const {
    double weight = parameters[0][0];
    Eigen::Map<Eigen::Matrix<double, 1, 1>> residual(residuals);
    residual(0, 0) = regularizer_ * (1 - weight);

    if (jacobians) {
      if (jacobians[0]) {
        Eigen::Map<Eigen::Matrix<double, 1, 1>> jacobian_weight(jacobians[0]);
        jacobian_weight(0, 0) = -regularizer_;
      }
    }

    return true;
  }
};

class GMRegularizationFactor : public RegularizationFactor {
 public:
  explicit GMRegularizationFactor(double regularizer) : RegularizationFactor(regularizer) {}

  virtual bool Evaluate(double const *const *parameters, double *residuals, double **jacobians) const {
    double weight = parameters[0][0];
    Eigen::Map<Eigen::Matrix<double, 1, 1>> residual(residuals);
    residual(0, 0) = regularizer_ * std::pow(std::sqrt(weight) - 1.0, 2);

    if (jacobians) {
      if (jacobians[0]) {
        Eigen::Map<Eigen::Matrix<double, 1, 1>> jacobian_weight(jacobians[0]);
        jacobian_weight(0, 0) = regularizer_ * (std::sqrt(weight) - 1.0) / std::sqrt(weight);
      }
    }

    return true;
  }
};

class LeclercRegularizationFactor : public RegularizationFactor {
 public:
  explicit LeclercRegularizationFactor(double regularizer) : RegularizationFactor(regularizer) {}

  virtual bool Evaluate(double const *const *parameters, double *residuals, double **jacobians) const {
    double weight = parameters[0][0];
    Eigen::Map<Eigen::Matrix<double, 1, 1>> residual(residuals);
    residual(0, 0) = regularizer_ * (weight * std::log(weight) - weight + 1);

    if (jacobians) {
      if (jacobians[0]) {
        Eigen::Map<Eigen::Matrix<double, 1, 1>> jacobian_weight(jacobians[0]);
        jacobian_weight(0, 0) = regularizer_ * std::log(weight);
      }
    }

    return true;
  }
};

class AdaptiveRegularizationFactor : public RegularizationFactor {
 public:
  explicit AdaptiveRegularizationFactor(double regularizer, double shape)
      : RegularizationFactor(regularizer), shape_(shape) {}

  virtual bool Evaluate(double const *const *parameters, double *residuals, double **jacobians) const {
    double weight = parameters[0][0];
    Eigen::Map<Eigen::Matrix<double, 1, 1>> residual(residuals);
    double a = abs(shape_ - 2.0) / shape_;
    double b = 1.0 - 0.5 * shape_;
    double c = shape_ / (shape_ - 2.0);

    double d = std::pow(weight, c);
    residual(0, 0) = regularizer_ * a * (b * d + 0.5 * shape_ * weight - 1.0);

    if (jacobians) {
      if (jacobians[0]) {
        Eigen::Map<Eigen::Matrix<double, 1, 1>> jacobian_weight(jacobians[0]);
        double e = shape_ * d * b;
        double f = weight * (shape_ - 2.0);

        jacobian_weight(0, 0) = regularizer_ * a * (e / f + 0.5 * shape_);
      }
    }

    return true;
  }

  double shape_;
};

class PriorFactor : public ceres::SizedCostFunction<1, 1> {
 public:
  PriorFactor() = delete;
  PriorFactor(double regularizer, double _prior) : regularizer_(regularizer), prior(_prior) {}
  virtual bool Evaluate(double const *const *parameters, double *residuals, double **jacobians) const {
    double weight = parameters[0][0];
    Eigen::Map<Eigen::Matrix<double, 1, 1>> residual(residuals);
    residual(0, 0) = fabs(regularizer_ * (prior - weight));

    if (jacobians) {
      if (jacobians[0]) {
        Eigen::Map<Eigen::Matrix<double, 1, 1>> jacobian_weight(jacobians[0]);
        if (prior < weight)
          jacobian_weight(0, 0) = regularizer_;
        else
          jacobian_weight(0, 0) = -regularizer_;
      }
    }

    return true;
  }

  double regularizer_;
  double prior;
};
