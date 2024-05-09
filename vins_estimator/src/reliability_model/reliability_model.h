//
// Created by wfram on 4/20/23.
//

#include <map>

#include <opencv2/opencv.hpp>

#ifndef VINS_RELIABILITY_MODEL_H
#define VINS_RELIABILITY_MODEL_H

enum SemanticPalette { ADE20K, CITYSCAPES, PASCAL_VOC };

enum SemanticGlobalLegend { ARBITRARY = 0, SURFACE, CAR, PERSON };

class SemanticMap {
 public:
  SemanticMap() = default;

  void initialize(const SemanticPalette palette) {
    switch (palette) {
      case ADE20K:
        createPaletteADE20K();
        break;
      case CITYSCAPES:
        createPaletteCityscapes();
        break;
      case PASCAL_VOC:
        createPalettePascalVoc();
        break;
      default:
        std::cerr << "Semantic palette is not implemented yet\n";
    }
  }

  void createPaletteADE20K();

  void createPaletteCityscapes();

  void createPalettePascalVoc();

  std::map<unsigned char, unsigned char> semantic_global_legend;
};

class ReliabilityModel {
 public:
  ReliabilityModel();

  void initializeModel(unsigned char label);

  void updateSemanticBelief(double prior);

  void updateGeometricBelief(double reprojection_error);

  void updateReliability(unsigned char label);

  void saturateProbability(double &probability);

  double getReliability() const;

  double getSemanticBelief() const;

 private:
  double reliability_;

  double semantic_belief_;
  double geometric_belief_;

  std::map<int, double> semantic_priors_;

  // TODO: configurable
  double belief_rate_ = 1.0;
  double complement_belief_rate_ = 0.0;
  double prior_rate_ = 0.25;
  double complement_prior_rate_ = 0.01;

  double stationary_reprojection_error_ = 5.0;

  double semantic_weight = 1.0;
  double geometric_weight = 0.0;
};

#endif  // VINS_RELIABILITY_MODEL_H
