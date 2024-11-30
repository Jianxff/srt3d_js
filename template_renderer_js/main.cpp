#include <srt3d/body.h>
#include <srt3d/model.h>
#include <srt3d/common.h>
#include <srt3d/renderer_geometry.h>

#include <Eigen/Geometry>
#include <filesystem>
#include <unordered_map>
#include <memory>
#include <string>

#include <emscripten/bind.h>
#include <emscripten/val.h>

namespace em = emscripten;

const Eigen::Matrix4f TRANSFORM_OPENCV_OPENGL = 
    (Eigen::Matrix4f() << 
        1, 0, 0, 0,
        0, -1, 0, 0,
        0, 0, -1, 0,
        0, 0, 0, 1 ).finished();


// convert emscripten val to original C++ type
template <typename T>
T read_emval(const em::val& val, const T default_val = T()) {
    if (val == em::val::undefined() || val == em::val::null())
        return default_val;
    return val.as<T>();
}


bool render_template(
    const std::string& name,
    const std::string& geometry_path, 
    const std::string& meta_path_,
    const float unit_in_meter, 
    const float shpere_radius, 
    const bool from_opengl
) {
    // make transform
    srt3d::Transform3fA geometry2body_pose(
        from_opengl ? TRANSFORM_OPENCV_OPENGL : Eigen::Matrix4f::Identity()
    );
    // make body geometry
    auto body_ptr_ = std::make_shared<srt3d::Body>(name, 
        geometry_path, unit_in_meter, true, true, geometry2body_pose);

    // set model
    auto meta_path = meta_path_;
    if(meta_path_.empty()) {
        meta_path = geometry_path;
        meta_path.append(".meta");
    }

    auto model_ptr_ = std::make_shared<srt3d::Model>(name, body_ptr_, meta_path, shpere_radius);

    if (!model_ptr_->GenerateModel()) return false;
    if (!model_ptr_->SaveModel()) return false;
    return true;
}

bool em_render_template(
    const std::string name,
    const std::string geometry_path,
    const std::string meat_path,
    const em::val config
) {
    const float unit_in_meter = read_emval<float>(config["unit_in_meter"], 1.0);
    const float shpere_radius = read_emval<float>(config["shpere_radius"], 0.8);
    const bool from_opengl = read_emval<bool>(config["from_opengl"], false);

    return render_template(name, geometry_path, meat_path, unit_in_meter, shpere_radius, from_opengl);
}


EMSCRIPTEN_BINDINGS(template_renderer) {
    em::function("em_render_template", &em_render_template);
}