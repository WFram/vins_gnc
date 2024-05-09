//
// Created by wfram on 11/9/22.
//

#ifndef VINS_GNC_H
#define VINS_GNC_H

#include "../estimator/parameters.h"

struct ShapeModel {
  double truncation_threshold_ = 0.9;
  double nan_shape_ = -10;

  double a_ = -8.0;
  double b_ = -2.0;
  double c_ = 1.0;
};

class ControlModel {
 public:
  ControlModel() = default;

  double getControlParameter(double shape) const { return a_ * shape * shape + b_ * shape + c_; }

 private:
  double a_ = -0.1;
  double b_ = -std::sqrt(3.0);
  double c_ = 2.0 * std::sqrt(2.0);
};

class SurrogateLoss {
 public:
  explicit SurrogateLoss(double fixed_shape)
      : fixed_shape_(fixed_shape), squared_fixed_shape_(fixed_shape * fixed_shape) {}

  virtual double initializeControlParameter(double max_residual) = 0;

  virtual double updateControlParameter() = 0;

  virtual void updateWeight(double residual, double mu, double &weight) = 0;

  virtual void updateShape(double reliability) = 0;

  virtual double getShape() = 0;

 protected:
  double fixed_shape_, squared_fixed_shape_;
};

class SurrogateL1 : public SurrogateLoss {
 public:
  explicit SurrogateL1(double fixed_shape) : SurrogateLoss(fixed_shape) {}

  double initializeControlParameter(double max_residual) override {
    throw std::runtime_error("Not implemented. Bye!\n");
  }

  double updateControlParameter() override { throw std::runtime_error("Not implemented. Bye!\n"); }

  void updateWeight(double residual, double mu, double &weight) override;

  void updateShape(double reliability) override { throw std::runtime_error("Not implemented. Bye!\n"); }

  double getShape() override { throw std::runtime_error("Not implemented. Bye!\n"); }
};

class SurrogateGM : public SurrogateLoss {
 public:
  explicit SurrogateGM(double fixed_shape) : SurrogateLoss(fixed_shape) {}

  double initializeControlParameter(double max_residual) override { return (2 * max_residual / squared_fixed_shape_); }

  double updateControlParameter() override { throw std::runtime_error("Not implemented. Bye!\n"); }

  void updateWeight(double residual, double mu, double &weight) override;

  void updateShape(double reliability) override { throw std::runtime_error("Not implemented. Bye!\n"); }

  double getShape() override { throw std::runtime_error("Not implemented. Bye!\n"); }
};

class SurrogateLeclerc : public SurrogateLoss {
 public:
  explicit SurrogateLeclerc(double fixed_shape) : SurrogateLoss(fixed_shape) {}

  double initializeControlParameter(double max_residual) override {
    throw std::runtime_error("Not implemented. Bye!\n");
  }

  double updateControlParameter() override { throw std::runtime_error("Not implemented. Bye!\n"); }

  void updateWeight(double residual, double mu, double &weight) override;

  void updateShape(double reliability) override { throw std::runtime_error("Not implemented. Bye!\n"); }

  double getShape() override { throw std::runtime_error("Not implemented. Bye!\n"); }
};

class SurrogateTLS : public SurrogateLoss {
 public:
  explicit SurrogateTLS(double fixed_shape) : SurrogateLoss(fixed_shape) {}

  double initializeControlParameter(double max_residual) override {
    throw std::runtime_error("Not implemented. Bye!\n");
  }

  double updateControlParameter() override { throw std::runtime_error("Not implemented. Bye!\n"); }

  void updateWeight(double residual, double mu, double &weight) override;

  void updateShape(double reliability) override { throw std::runtime_error("Not implemented. Bye!\n"); }

  double getShape() override { throw std::runtime_error("Not implemented. Bye!\n"); }
};

class SurrogateAdaptive : public SurrogateLoss {
 public:
  explicit SurrogateAdaptive(double fixed_shape) : SurrogateLoss(fixed_shape) {}

  double initializeControlParameter(double max_residual) override {
    throw std::runtime_error("Not implemented. Bye!\n");
  }

  double updateControlParameter() override;

  void updateWeight(double residual, double mu, double &weight) override;

  void updateShape(double reliability) final;

  double getShape() final;

 private:
  double shape_;

  ShapeModel shape_model_;

  ControlModel control_model_;
};

std::unique_ptr<SurrogateLoss> createSurrogateLoss(SurrogateLossType type, double fixed_shape);

#endif  // VINS_GNC_H
