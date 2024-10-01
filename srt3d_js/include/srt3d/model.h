// SPDX-License-Identifier: MIT
// Copyright (c) 2021 Manuel Stoiber, German Aerospace Center (DLR)

#ifndef OBJECT_TRACKING_INCLUDE_SRT3D_MODEL_H_
#define OBJECT_TRACKING_INCLUDE_SRT3D_MODEL_H_

// #include <omp.h>
#include <srt3d/body.h>
#include <srt3d/common.h>
// #include <srt3d/normal_renderer.h>
// #include <srt3d/renderer_geometry.h>

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <array>
#include <atomic>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <memory>
#include <opencv2/opencv.hpp>
#include <random>
#include <string>
#include <vector>

namespace srt3d {

// Class that stores a sparse viewpoint model of a body that consists of views
// with multiple contour points. It includes all functionality to generate,
// save, and load the model
class Model {
 private:
  // Some constants
  static constexpr int kVersionID = 2;
  static constexpr int kContourNormalApproxRadius = 3;
  static constexpr int kMinContourLength = 15;
  static constexpr int kImageSizeSafetyBoundary = 20;

  // Struct with operator that compares two Vector3f and checks if v1 < v2
  struct CompareSmallerVector3f {
    bool operator()(const Eigen::Vector3f &v1,
                    const Eigen::Vector3f &v2) const {
      return v1[0] < v2[0] || (v1[0] == v2[0] && v1[1] < v2[1]) ||
             (v1[0] == v2[0] && v1[1] == v2[1] && v1[2] < v2[2]);
    }
  };

 public:
  using PointData = struct PointData {
    Eigen::Vector3f center_f_body;
    Eigen::Vector3f normal_f_body;
    float foreground_distance = 0.0f;
    float background_distance = 0.0f;
  };

  using TemplateView = struct TemplateView {
    std::vector<PointData> data_points;
    Eigen::Vector3f orientation;  // points from camera to body center
  };

  // Constructors and setup methods
  // Model(const std::string &name, std::shared_ptr<Body> body_ptr,
  //       const std::filesystem::path &directory, const std::string &filename,
  //       float sphere_radius = 0.8, int n_divides = 4, int n_points = 200,
  //       bool use_random_seed = true, int image_size = 2000);
  Model(const std::string &name, std::shared_ptr<Body> body_ptr,
      const std::string &meta_file);
  bool SetUp();

  // Main methods
  bool GetClosestTemplateView(const Transform3fA &body2camera_pose,
                              const TemplateView **closest_template_view) const;

  // Getters
  const std::string &name() const;
  std::shared_ptr<Body> body_ptr() const;
  bool set_up() const;

 private:
  // Helper methods for model set up
  // bool GenerateModel();
  bool LoadModel();

  // Model data
  std::vector<TemplateView> template_views_;

  // Data
  std::string name_{};
  std::shared_ptr<Body> body_ptr_ = nullptr;
  std::string meta_file_{};
  int n_points_ = 200;
  bool set_up_ = false;
};

}  // namespace srt3d

#endif  // OBJECT_TRACKING_INCLUDE_SRT3D_MODEL_H_
