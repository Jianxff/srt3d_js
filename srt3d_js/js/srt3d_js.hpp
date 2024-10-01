#pragma once 

#include <srt3d/body.h>
#include <srt3d/common.h>
#include <srt3d/region_modality.h>
// #include <srt3d/renderer_geometry.h>
#include <srt3d/tracker.h>

#include "virtual_camera.hpp"

#include <Eigen/Geometry>
#include <filesystem>
#include <unordered_map>
#include <memory>
#include <string>

#include <emscripten/bind.h>
#include <emscripten/val.h>

namespace em = emscripten;

// convert emscripten val to original C++ type
template <typename T>
T read_emval(const em::val& val, const T default_val = T()) {
    if (val == em::val::undefined() || val == em::val::null())
        return default_val;
    return val.as<T>();
}

// convert emscripten val to Eigen::Matrix
template <typename T>
Eigen::MatrixX<T> read_emval_matrix(const em::val& val, const int rows, const int cols) {
    Eigen::MatrixX<T> matrix(rows, cols);
    for(int i = 0; i < rows; i++) {
        for(int j = 0; j < cols; j++) {
            matrix(i, j) = read_emval<T>(val[i][j]);
        }
    }
    return matrix;
}

// convert Eigen::Matrix to emscripten val
template <typename T>
em::val convert_matrix_emval(const Eigen::MatrixX<T>& matrix) {
    em::val array = em::val::array();
    for(int i = 0; i < matrix.rows(); i++) {
        em::val row = em::val::array();
        for(int j = 0; j < matrix.cols(); j++)
            row.set(j, matrix(i, j));
        array.set(i, row);
    }
    return array;
}

// vector to emscripten val
template <typename T>
em::val convert_vector_emval(const std::vector<T>& vec) {
    em::val array = em::val::array();
    for(int i = 0; i < vec.size(); i++) {
        array.set(i, vec[i]);
    }
    return array;
}

// calculate pose uv
cv::Point2i calculate_pose_uv(const Eigen::Matrix4f& pose, const Eigen::Matrix3f& K);

class Canvas{
public:
    Canvas(const int width, const int height);
    const cv::Mat& mat() const;
    const em::val matData() const;
    void set(const cv::Mat& mat);
protected:
    cv::Mat mat_;
};


class RegionModel{
public:
    // constructor
    RegionModel(const std::string& name, 
        const std::string& meta_path,
        const em::val config
    );
    // setup tracker
    void Setup();

    // variables
    const std::string name_;
    std::shared_ptr<srt3d::Body> body_ptr_ = nullptr;
    std::shared_ptr<srt3d::Model> model_ptr_ = nullptr;
    std::shared_ptr<srt3d::RegionModality> region_modality_ptr_ = nullptr;
    float threshold_on_init_;
    float threshold_on_track_;
    float kl_threshold_ = 1.0;
    cv::Mat visualize_image_;

    // states
    float conf_ = 0.0;
    Eigen::Matrix4f last_pose_ = Eigen::Matrix4f::Identity();
    int state_ = 1;
    cv::Point2i uv_;

    // methods
    // void set_target_rotation(const Eigen::Matrix3f& rot);
    void set_kl_threshold(const float kl_threshold);
    void reset_pose(const Eigen::Matrix4f& pose = Eigen::Matrix4f::Identity());
    const Eigen::Matrix4f get_pose() const;
    const Eigen::Matrix4f get_pose_gl() const;
    bool validate();
    float valid_percentage();

    // *** EMSCRIPTEN BINDINGS ***
    void em_set_pose(em::val pose = em::val::undefined());
    void em_set_pose_gl(em::val pose = em::val::undefined());
    const em::val em_get_pose() const;
    const em::val em_get_pose_gl() const;
    const em::val em_get_pose_uv() const;
    const std::shared_ptr<Canvas> em_get_visualize_image() const;
    // *** EMSCRIPTEN BINDINGS ***

}; // class Model


class RegionTracker{
public:
    RegionTracker(
        const int imwidth, const int imheight, 
        const em::val config
    );
    // add region model
    void add_model(const std::shared_ptr<RegionModel>& region_model);
    // track
    void track(const std::shared_ptr<Canvas>& frame);
    // setup
    void setup();
    // delete
    void terminate();

    int imwidth_;
    int imheight_;
    int focal_;
    Eigen::Matrix3f K_;

    std::vector<std::shared_ptr<RegionModel>> models_;
    std::shared_ptr<srt3d::Tracker> tracker_ptr_;
    std::shared_ptr<srt3d::VirtualCamera> camera_ptr_;

    // *** EMSCRIPTEN BINDINGS ***
    const em::val em_get_K() const;
    const em::val em_get_models() const;
    // *** EMSCRIPTEN BINDINGS ***

}; // class tracker


// class RegionRenderer{
// public:
//     RegionRenderer(const std::shared_ptr<RegionTracker>& region_tracker_ptr);
//     py::array_t<uint8_t> render();

//     std::shared_ptr<RegionTracker> region_tracker_;
//     std::shared_ptr<srt3d::RendererGeometry> renderer_geometry_ptr_;
//     std::shared_ptr<srt3d::NormalViewer> viewer_ptr_;
//     std::shared_ptr<srt3d::VirtualCamera> camera_ptr_;
// }; // class renderer


