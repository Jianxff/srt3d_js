#include "srt3d_js.hpp"
#include <filesystem>
#include <chrono>

const Eigen::Matrix4f TRANSFORM_OPENCV_OPENGL = 
    (Eigen::Matrix4f() << 
        1, 0, 0, 0,
        0, -1, 0, 0,
        0, 0, -1, 0,
        0, 0, 0, 1 ).finished();


cv::Point2i calculate_pose_uv(const Eigen::Matrix4f& pose, const Eigen::Matrix3f& K) {
    Eigen::Vector3f p = pose.block<3, 1>(0, 3);
    Eigen::Vector3f puv = K * p;
    return cv::Point2i(puv[0] / puv[2], puv[1] / puv[2]);
}


RegionModel::RegionModel(const std::string& name,
    const std::string& meta_path_,
    const em::val config
) : name_(name)
{
    // args
    // float unit_in_meter = read_emval<float>(config["unit_in_meter"], 1.0);
    // float shpere_radius = read_emval<float>(config["shpere_radius"], 0.8);
    // bool from_opengl = read_emval<bool>(config["from_opengl"], false);
    float threshold_on_init = read_emval<float>(config["threshold_on_init"], 0.0);
    float threshold_on_track = read_emval<float>(config["threshold_on_track"], 0.0);
    float kl_threshold = read_emval<float>(config["kl_threshold"], 1.0);
    threshold_on_init_ = threshold_on_init;
    threshold_on_track_ = threshold_on_track;
    kl_threshold_ = kl_threshold;

    // make transform
    // srt3d::Transform3fA geometry2body_pose(
    //     (from_opengl ? TRANSFORM_OPENCV_OPENGL : Eigen::Matrix4f::Identity())
    // );
    // make body geometry
    body_ptr_ = std::make_shared<srt3d::Body>(name, "", 1.0, true, true);

    // set model
    model_ptr_ = std::make_shared<srt3d::Model>(name, body_ptr_, meta_path_);
    // reset to identity
    reset_pose(Eigen::Matrix4f::Identity());
}

void RegionModel::Setup() {
    model_ptr_->SetUp();
}


// void RegionModel::set_target_rotation(const Eigen::Matrix3f& rot) {
//     if (region_modality_ptr_ != nullptr) 
//         region_modality_ptr_->set_target_rotation(rot);
// }

void RegionModel::set_kl_threshold(const float kl_threshold) {
    kl_threshold_ = kl_threshold;
    region_modality_ptr_->set_kl_threshold(kl_threshold);
}

void RegionModel::reset_pose(const Eigen::Matrix4f& pose) {
    srt3d::Transform3fA body2world_pose(pose);
    body_ptr_->set_body2world_pose(body2world_pose);
    last_pose_ = pose;
}


const Eigen::Matrix4f RegionModel::get_pose() const{
    return body_ptr_->body2world_pose().matrix();
}


const Eigen::Matrix4f RegionModel::get_pose_gl() const{
    Eigen::Matrix4f pose_cv = body_ptr_->body2world_pose().matrix();
    Eigen::Matrix4f pose_gl = TRANSFORM_OPENCV_OPENGL * pose_cv;
    return pose_gl;
}


bool RegionModel::validate() {
    // while tracking
    if (state_ == 0) { // stage: init
        if (conf_ >= threshold_on_track_) last_pose_ = get_pose();
        else {
            reset_pose(last_pose_);
            state_ = 1;
        }
    } else if (state_ == 1) { // stage: on tracking
        if (conf_ >= threshold_on_init_) {
            last_pose_ = get_pose();
            state_ = 0;
        } else reset_pose(last_pose_); 
    }

    return state_ == 0;
}

float RegionModel::valid_percentage() {
    int len_data_lines = region_modality_ptr_->len_data_lines();
    return (float)len_data_lines / (float)region_modality_ptr_->get_n_lines();
}



RegionTracker::RegionTracker(
    const int imwidth, const int imheight, 
    const em::val config
) : imwidth_(imwidth), imheight_(imheight) 
{
    // args
    float fx_ = read_emval<float>(config["fx"], 0.0);
    float fy_ = read_emval<float>(config["fy"], 0.0);
    float cx_ = read_emval<float>(config["cx"], 0.0);
    float cy_ = read_emval<float>(config["cy"], 0.0);
    int corr_update_iter = read_emval<int>(config["corr_update_iter"], 7);
    int pose_update_iter = read_emval<int>(config["pose_update_iter"], 2);

    // make tracker
    tracker_ptr_ = std::make_shared<srt3d::Tracker>("tracker");
    tracker_ptr_->set_n_corr_iterations(corr_update_iter);
    tracker_ptr_->set_n_update_iterations(pose_update_iter);

    /// set camera =======================================================
    // make virtual camera
    float fx = fx_, fy = fy_, cx = cx_, cy = cy_;
    camera_ptr_ = std::make_shared<srt3d::VirtualCamera>("vcam");

    // set intrinsics
    if (cx * cy == 0.0) {
        cx = (float)imwidth / 2.0f;
        cy = (float)imheight / 2.0f;
    }
    if (fx == 0.0) {
        fx = std::max((float)imwidth, (float)imheight);
        fy = fx;
    }
    if (fy == 0.0) {
        fy = fx;
    }

    // std::cout << "Camera intrinsics: " << fx << " " << fy << " " << cx << " " << cy << std::endl;

    srt3d::Intrinsics intrinsics {
        fx, fy, cx, cy, imwidth, imheight
    };
    camera_ptr_->set_intrinsics(intrinsics);
    // set K
    K_ << fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0;
}

void RegionTracker::add_model(const std::shared_ptr<RegionModel>& region_model) {
    // set region model
    auto region_modality_ptr = std::make_shared<srt3d::RegionModality>(
        region_model->name_, region_model->body_ptr_, region_model->model_ptr_, camera_ptr_);
    tracker_ptr_->AddRegionModality(region_modality_ptr);
    region_model->region_modality_ptr_ = region_modality_ptr;
    region_model->region_modality_ptr_->set_kl_threshold(region_model->kl_threshold_);
    models_.push_back(std::move(region_model));
}


void RegionTracker::setup() {
    if (!tracker_ptr_->set_up()) {
        tracker_ptr_->SetUpTracker();
    }
}

void RegionTracker::track(const std::shared_ptr<Canvas>& frame) 
{
    if (!tracker_ptr_->set_up()) {
        tracker_ptr_->SetUpTracker();
    }
    // get rgb image
    cv::Mat cv_image = frame->mat();
    cv::Mat cv_image_bgr;
    cv::cvtColor(cv_image, cv_image_bgr, cv::COLOR_RGBA2BGR);
    // push image
    camera_ptr_->PushImage(cv_image_bgr);
    // main tracking loop
    tracker_ptr_->TrackIter();
    // auto st = std::chrono::steady_clock::now();

    // evaluate distribution
    std::vector<float> conf_list = tracker_ptr_->EvaluateDistribution();
    for(int i = 0; i < conf_list.size(); i++) {
        models_[i]->conf_ = conf_list[i];
        models_[i]->validate();
        models_[i]->uv_ = calculate_pose_uv(models_[i]->get_pose(), K_);
    }

    // double ms = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - st).count();
    // std::cout << "Evaluation time: " << ms << "ms" << std::endl;
}

void RegionTracker::terminate() {
    for(auto &model : models_) {
        model.reset();
    }
    models_.clear();
    tracker_ptr_.reset();
    camera_ptr_.reset();
}



Canvas::Canvas(int width, int height) {
    mat_ = cv::Mat(height, width, CV_8UC4);
}

const cv::Mat& Canvas::mat() const {
    return mat_;
}

const emscripten::val Canvas::matData() const{
    return emscripten::val(emscripten::memory_view<unsigned char>((mat_.total()*mat_.elemSize())/sizeof(unsigned char),
                            (unsigned char*)mat_.data));
}

void Canvas::set(const cv::Mat& mat) {
    mat_ = mat.clone();
}


// *** EMSCRIPTEN BINDINGS ***
void RegionModel::em_set_pose(em::val pose) {
    Eigen::Matrix4f pose_ = read_emval_matrix<float>(pose, 4, 4);
    reset_pose(pose_);
}

void RegionModel::em_set_pose_gl(em::val pose) {
    Eigen::Matrix4f pose_ = read_emval_matrix<float>(pose, 4, 4);
    pose_ = TRANSFORM_OPENCV_OPENGL * pose_;
    reset_pose(pose_);
}

const em::val RegionModel::em_get_pose() const {
    auto pose = get_pose();
    return convert_matrix_emval<float>(pose);
}

const em::val RegionModel::em_get_pose_gl() const {
    auto pose = get_pose_gl();
    return convert_matrix_emval<float>(pose);
}

const em::val RegionModel::em_get_pose_uv() const {
    auto uv = em::val::object();
    uv.set("u", uv_.x);
    uv.set("v", uv_.y);
    return uv;
}

const em::val RegionTracker::em_get_K() const{
    return convert_matrix_emval<float>(K_);
}

const em::val RegionTracker::em_get_models() const {
    return convert_vector_emval<std::shared_ptr<RegionModel>>(models_);
}

const std::shared_ptr<Canvas> RegionModel::em_get_visualize_image() const{
    region_modality_ptr_->VisualizeCorrespondences(0);
    cv::Mat img = region_modality_ptr_->visualize_image_;
    cv::cvtColor(img, img, cv::COLOR_BGR2RGBA);
    if (img.empty()) {
        img = cv::Mat(1, 1, CV_8UC4, cv::Scalar(0, 0, 0, 0));
        std::cerr << "Visualization is disabled" << std::endl;
    }
    auto frame = std::make_shared<Canvas>(img.cols, img.rows);
    frame->set(img);
    return frame;
}

EMSCRIPTEN_BINDINGS(websrt3d)
{
    // emscripten::constant("description", "Javascript binding for SRT3D");
    emscripten::constant("version", 0);

    // frame
    em::class_<Canvas>("Canvas")
        .smart_ptr_constructor("shared_ptr<Canvas>",
            &std::make_shared<Canvas, int, int>
        )
        .property("data", &Canvas::matData);

    // model
    em::class_<RegionModel>("Model")
        .smart_ptr_constructor("shared_ptr<Model>",
            &std::make_shared<RegionModel, 
                const std::string&, const std::string&, const em::val>
        )
        .function("set_pose", &RegionModel::em_set_pose)
        .function("set_pose_gl", &RegionModel::em_set_pose_gl)
        .property("name", &RegionModel::name_)
        .property("conf", &RegionModel::conf_)
        .property("pose", &RegionModel::em_get_pose)
        .property("pose_gl", &RegionModel::em_get_pose_gl)
        .property("pose_uv", &RegionModel::em_get_pose_uv)
        .property("prob_image", &RegionModel::em_get_visualize_image);

    // tracker
    em::class_<RegionTracker>("Tracker")
        .smart_ptr_constructor("shared_ptr<Tracker>",
            &std::make_shared<RegionTracker, 
                const int, const int, const em::val>
        )
        .function("add_model", &RegionTracker::add_model)
        .function("setup", &RegionTracker::setup)
        .function("update", &RegionTracker::track)
        .property("K", &RegionTracker::em_get_K)
        .property("models", &RegionTracker::em_get_models);

    em::register_vector<std::shared_ptr<RegionModel>>("ModelList");
}