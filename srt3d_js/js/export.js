const SRT3D_JS_CONFIG = {
    // Camera parameters
    fx: 0.0,
    fy: 0.0,
    cx: 0.0,
    cy: 0.0,
    // Tracking parameters
    kl_threshold: 1.0,
    threshold_on_init: 0.8,
    threshold_on_track: 0.5,
    corr_update_iter: 7,
    pose_update_iter: 2,
}

export default initSRT3D;