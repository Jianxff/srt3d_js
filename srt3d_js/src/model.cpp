// SPDX-License-Identifier: MIT
// Copyright (c) 2021 Manuel Stoiber, German Aerospace Center (DLR)

#include <srt3d/model.h>

namespace srt3d {

// Model::Model(const std::string &name, std::shared_ptr<Body> body_ptr,
//              const std::filesystem::path &directory,
//              const std::string &filename, float sphere_radius, int n_divides,
//              int n_points, bool use_random_seed, int image_size)
//     : name_{name},
//       body_ptr_{std::move(body_ptr)},
//       directory_{directory},
//       filename_{filename},
//       sphere_radius_{sphere_radius},
//       n_divides_{n_divides},
//       n_points_{n_points},
//       use_random_seed_{use_random_seed},
//       image_size_{image_size} {}

Model::Model(const std::string &name, std::shared_ptr<Body> body_ptr,
    const std::string &meta_file)
    : name_{name},
      body_ptr_{std::move(body_ptr)},
      meta_file_{meta_file} {}

bool Model::SetUp() {
  set_up_ = false;
  if (!LoadModel()) {
      return false;
  }
  set_up_ = true;
  return true;
}


bool Model::GetClosestTemplateView(
    const Transform3fA &body2camera_pose,
    const TemplateView **closest_template_view) const {
  if (!set_up_) {
    std::cerr << "Set up model " << name_ << " first" << std::endl;
    return false;
  }

  Eigen::Vector3f orientation{
      body2camera_pose.rotation().inverse() *
      body2camera_pose.translation().matrix().normalized()};

  float closest_dot = -1.0f;
  for (auto &template_view : template_views_) {
    float dot = orientation.dot(template_view.orientation);
    if (dot > closest_dot) {
      *closest_template_view = &template_view;
      closest_dot = dot;
    }
  }
  return true;
}

const std::string &Model::name() const { return name_; }

std::shared_ptr<Body> Model::body_ptr() const { return body_ptr_; }

bool Model::set_up() const { return set_up_; }


bool Model::LoadModel() {
  std::filesystem::path data_path{meta_file_};
  std::ifstream data_ifs{data_path, std::ios::in | std::ios::binary};
  if (!data_ifs.is_open() || data_ifs.fail()) {
    data_ifs.close();
    std::cout << "Could not open model file " << data_path << std::endl;
    return false;
  }
  // Check if model file has correct parameters
  int version_id_file;
  float sphere_radius_file;
  int n_divides_file;
  int n_points_file;
  bool use_random_seed_file;
  int image_size_file;
  data_ifs.read((char *)(&version_id_file), sizeof(version_id_file));
  data_ifs.read((char *)(&sphere_radius_file), sizeof(sphere_radius_file));
  data_ifs.read((char *)(&n_divides_file), sizeof(n_divides_file));
  data_ifs.read((char *)(&n_points_file), sizeof(n_points_file));
  data_ifs.read((char *)(&use_random_seed_file), sizeof(use_random_seed_file));
  data_ifs.read((char *)(&image_size_file), sizeof(image_size_file));
  // if (version_id_file != kVersionID || /*sphere_radius_file != sphere_radius_ ||*/
  //     n_divides_file != n_divides_ || n_points_file < n_points_ ||
  //     use_random_seed_file != use_random_seed_ ||
  //     image_size_file != image_size_) {
  //   std::cout << "Model file " << data_path
  //             << " was generated using different model parameters" << std::endl;
  //   return false;
  // }

  // Check if model file has correct geometry data
  float geometry_unit_in_meter;
  bool geometry_counterclockwise;
  bool geometry_enable_culling;
  float maximum_body_diameter;
  Transform3fA geometry2body_pose;
  data_ifs.read((char *)(&geometry_unit_in_meter),
                sizeof(geometry_unit_in_meter));
  data_ifs.read((char *)(&geometry_counterclockwise),
                sizeof(geometry_counterclockwise));
  data_ifs.read((char *)(&geometry_enable_culling),
                sizeof(geometry_enable_culling));
  data_ifs.read((char *)(&maximum_body_diameter),
                sizeof(maximum_body_diameter));
  data_ifs.read((char *)(geometry2body_pose.data()),
                sizeof(geometry2body_pose));

  // if (geometry_unit_in_meter != body_ptr_->geometry_unit_in_meter() ||
  //     geometry_counterclockwise != body_ptr_->geometry_counterclockwise() ||
  //     geometry_enable_culling != body_ptr_->geometry_enable_culling() ||
  //     // maximum_body_diameter != body_ptr_->maximum_body_diameter() ||
  //     geometry2body_pose.matrix() != body_ptr_->geometry2body_pose().matrix()) {
  //   std::cout << "Model file " << data_path
  //             << " was generated using different body parameters" << std::endl;
  //   return false;
  // }
  body_ptr_->set_geometry_unit_in_meter(geometry_unit_in_meter);
  body_ptr_->set_geometry_counterclockwise(geometry_counterclockwise);
  body_ptr_->set_geometry_enable_culling(geometry_enable_culling);
  body_ptr_->set_maximum_body_diameter(maximum_body_diameter);
  body_ptr_->set_geometry2body_pose(geometry2body_pose);

  // Load template view data
  int n_template_views;
  data_ifs.read((char *)(&n_template_views), sizeof(n_template_views));
  template_views_.clear();
  template_views_.reserve(n_template_views);
  for (int i = 0; i < n_template_views; i++) {
    TemplateView tv;
    tv.data_points.resize(n_points_);
    data_ifs.read((char *)(tv.data_points.data()),
                  n_points_ * sizeof(PointData));
    data_ifs.read((char *)(tv.orientation.data()), sizeof(tv.orientation));
    template_views_.push_back(std::move(tv));
  }
  data_ifs.close();


  // std::cout << "Model " << name_ << " loaded from " << data_path << std::endl;
  // std::cout << "  - Sphere radius: " << sphere_radius_file << std::endl;
  // std::cout << "  - Number of template views: " << n_template_views << std::endl;
  // std::cout << "  - Number of points per template view: " << n_points_ << std::endl;
  // std::cout << "  - Use random seed: " << use_random_seed_file << std::endl;
  // std::cout << template_views_[0].data_points[0].center_f_body << std::endl;
  // std::cout << template_views_[0].data_points[0].normal_f_body << std::endl;
  // std::cout << template_views_[0].data_points[0].foreground_distance << std::endl;
  // std::cout << template_views_[0].data_points[0].background_distance << std::endl;
  // std::cout << template_views_[0].orientation << std::endl;


  return true;
}

}  // namespace srt3d
