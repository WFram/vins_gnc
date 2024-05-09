//
// Created by wfram on 4/22/23.
//

#include "surrogate_loss.h"

void SurrogateL1::updateWeight(double residual, double mu, double &weight) {
  const double mu_c2 = mu * squared_fixed_shape_;
  weight = std::sqrt(mu_c2 / 4 * residual);
}

void SurrogateGM::updateWeight(double residual, double mu, double &weight) {
  const double mu_c2 = mu * squared_fixed_shape_;
  weight = std::pow(mu_c2 / (residual + mu_c2), 2);
}

void SurrogateLeclerc::updateWeight(double residual, double mu, double &weight) {
  const double mu_c2 = mu * squared_fixed_shape_;
  weight = std::exp(-(residual / mu_c2));
}

void SurrogateTLS::updateWeight(double residual, double mu, double &weight) {
  double right_boarder = ((mu + 1.0f) / mu) * squared_fixed_shape_;
  double left_boarder = (mu / (mu + 1.0f)) * squared_fixed_shape_;
  if (residual > right_boarder)
    weight = 0.0f;
  else if (residual > left_boarder && residual <= right_boarder)
    weight = (fixed_shape_ / sqrt(residual)) * sqrt(mu * (mu + 1.0f)) - mu;
  else
    weight = 1.0f;
}

void SurrogateAdaptive::updateWeight(double residual, double mu, double &weight) {
  const double mu_c2 = mu * squared_fixed_shape_;
  const double inv_mu_c2 = 1 / mu_c2;
  weight = inv_mu_c2 * std::pow((residual / mu_c2) / abs(shape_ - 2.0) + 1.0, 0.5 * shape_ - 1.0);
}

void SurrogateAdaptive::updateShape(double reliability) {
  double unreliability = 1.0 - reliability;
  double squared_unreliability = unreliability * unreliability;

  if (unreliability <= shape_model_.truncation_threshold_)
    shape_ = shape_model_.a_ * squared_unreliability + shape_model_.b_ * unreliability + shape_model_.c_;
  else
    shape_ = shape_model_.nan_shape_;
}

double SurrogateAdaptive::getShape() {
  return shape_;
}

double SurrogateAdaptive::updateControlParameter() {
  double control_parameter;
  if (shape_ <= 1.0)
    control_parameter = control_model_.getControlParameter(shape_);
  else
    control_parameter = 1.0;

  return control_parameter;
}

std::unique_ptr<SurrogateLoss> createSurrogateLoss(SurrogateLossType type, double fixed_shape) {
  switch (type) {
    case SurrogateLossType::L1:
      return std::make_unique<SurrogateL1>(fixed_shape);
    case SurrogateLossType::GM:
      return std::make_unique<SurrogateGM>(fixed_shape);
    case SurrogateLossType::LECLERC:
      return std::make_unique<SurrogateLeclerc>(fixed_shape);
    case SurrogateLossType::TLS:
      return std::make_unique<SurrogateTLS>(fixed_shape);
    case SurrogateLossType::ADAPTIVE:
      return std::make_unique<SurrogateAdaptive>(fixed_shape);
    default:
      std::cerr << "Wrong loss type\n";
  }
}