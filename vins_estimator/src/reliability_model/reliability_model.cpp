//
// Created by wfram on 4/20/23.
//

#include "reliability_model.h"

void SemanticMap::createPaletteADE20K() {
  semantic_global_legend.insert({0, SemanticGlobalLegend::SURFACE});
  semantic_global_legend.insert({3, SemanticGlobalLegend::SURFACE});
  semantic_global_legend.insert({5, SemanticGlobalLegend::SURFACE});
  semantic_global_legend.insert({12, SemanticGlobalLegend::PERSON});
  semantic_global_legend.insert({20, SemanticGlobalLegend::CAR});
}

void SemanticMap::createPaletteCityscapes() {
  semantic_global_legend.insert({0, SemanticGlobalLegend::SURFACE});
  semantic_global_legend.insert({1, SemanticGlobalLegend::SURFACE});
  semantic_global_legend.insert({2  , SemanticGlobalLegend::SURFACE});
  semantic_global_legend.insert({3, SemanticGlobalLegend::SURFACE});
  semantic_global_legend.insert({11, SemanticGlobalLegend::PERSON});
  semantic_global_legend.insert({13, SemanticGlobalLegend::CAR});
  semantic_global_legend.insert({14, SemanticGlobalLegend::CAR});
  semantic_global_legend.insert({15, SemanticGlobalLegend::CAR});
  semantic_global_legend.insert({16, SemanticGlobalLegend::CAR});
  semantic_global_legend.insert({17, SemanticGlobalLegend::CAR});
}

void SemanticMap::createPalettePascalVoc() {
  semantic_global_legend.insert({6, SemanticGlobalLegend::CAR});
  semantic_global_legend.insert({7, SemanticGlobalLegend::CAR});
  semantic_global_legend.insert({14, SemanticGlobalLegend::CAR});
  semantic_global_legend.insert({15, SemanticGlobalLegend::PERSON});
}

ReliabilityModel::ReliabilityModel() {
  semantic_priors_.insert({SemanticGlobalLegend::ARBITRARY, 0.05});
  semantic_priors_.insert({SemanticGlobalLegend::SURFACE, 0.05});
  semantic_priors_.insert({SemanticGlobalLegend::CAR, 0.95});
  semantic_priors_.insert({SemanticGlobalLegend::PERSON, 0.95});
}

void ReliabilityModel::initializeModel(unsigned char label) {
  semantic_belief_ = semantic_priors_[label];
  reliability_ = 1.0 - semantic_weight * semantic_belief_;
}

void ReliabilityModel::saturateProbability(double &probability) {
  if (probability > 1.0) probability = 1.0;
  if (probability < 0.0) probability = 0.0;
}

void ReliabilityModel::updateGeometricBelief(const double reprojection_error) {
  double absolute_reprojection_error = abs(reprojection_error);

  geometric_belief_ = absolute_reprojection_error / stationary_reprojection_error_;

  // TODO: it may decrease the robustness
  saturateProbability(geometric_belief_);
}

void ReliabilityModel::updateSemanticBelief(const double prior) {
  double prior_rate = prior_rate_;
  if (prior > semantic_belief_) {
    prior_rate = 1.0 - prior;
  }

  semantic_belief_ = prior * (belief_rate_ * semantic_belief_ + complement_belief_rate_ * (1.0 - semantic_belief_));
  double normalizer = 1.0 / (semantic_belief_ + prior_rate * prior + complement_prior_rate_ * (1.0 - prior));
  semantic_belief_ *= normalizer;
}

void ReliabilityModel::updateReliability(const unsigned char label) {
  double semantic_prior = semantic_priors_[label];

  if (abs(semantic_prior - semantic_belief_) <= 1e-5) {
    return;
  }

  updateSemanticBelief(semantic_prior);

  const double dummy_reprojection_error = 0.0;
  updateGeometricBelief(dummy_reprojection_error);

  reliability_ = 1.0 - semantic_weight * semantic_belief_ + geometric_weight * geometric_belief_;
}

double ReliabilityModel::getReliability() const { return reliability_; }

double ReliabilityModel::getSemanticBelief() const { return semantic_belief_; }