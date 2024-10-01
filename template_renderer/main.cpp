#include <srt3d/body.h>
#include <srt3d/model.h>
#include <srt3d/common.h>
#include <srt3d/renderer_geometry.h>

#include <Eigen/Geometry>
#include <filesystem>
#include <unordered_map>
#include <memory>
#include <string>

const Eigen::Matrix4f TRANSFORM_OPENCV_OPENGL = 
    (Eigen::Matrix4f() << 
        1, 0, 0, 0,
        0, -1, 0, 0,
        0, 0, -1, 0,
        0, 0, 0, 1 ).finished();


bool render_template(
    const std::string& name,
    const std::string& geometry_path, 
    const std::string& meta_path_,
    const float unit_in_meter, 
    const float shpere_radius, 
    const bool from_opengl,
    const Eigen::Matrix4f& geometry2body = Eigen::Matrix4f::Identity()
) {
    Eigen::Matrix4f geometry2body_ = geometry2body;
    if (from_opengl) {
        geometry2body_ = TRANSFORM_OPENCV_OPENGL * geometry2body;
    }
    // make transform
    srt3d::Transform3fA geometry2body_pose(geometry2body_);
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



int main(int argc, char const *argv[])
{
    // arg check
    if(argc < 4 or argc > 7) {
        std::cerr << "Usage: " << argv[0] << " <name> <geometry_path> <meta_path> <unit_in_meter = 1.0> <shpere_radius = 0.8> <from_opengl = false>" << std::endl;
        return 1;
    }
    const std::string name = argv[1];
    const std::string geometry_path = argv[2];
    const std::string meta_path = argv[3];

    float unit_in_meter = 1.0;
    if (argc >= 5) unit_in_meter = std::stof(argv[4]);
    
    float shpere_radius = 0.8;
    if (argc >= 6) shpere_radius = std::stof(argv[5]);

    bool from_opengl = false;
    if (argc == 7) from_opengl = std::stoi(argv[6]);

    // render template
    render_template(
        name, 
        geometry_path, 
        meta_path, 
        unit_in_meter, 
        shpere_radius, 
        from_opengl
    );

    return 0;
}