#pragma once
#include <Eigen/Dense>
#include <vector>
#include <string>

// Single Lidar point with Doppler
struct LidarPoint {
    double x;
    double y;
    double z;
    double v_radial;
};

// Skew-symmetric matrix
inline Eigen::Matrix3d skew(const Eigen::Vector3d& v) {
    Eigen::Matrix3d m;
    m << 0.0, -v.z(), v.y(),
         v.z(), 0.0, -v.x(),
        -v.y(), v.x(), 0.0;
    return m;
}

// SO(3) exp
inline Eigen::Matrix3d exp_so3(const Eigen::Vector3d& omega) {
    double theta = omega.norm();
    if (theta < 1e-12) return Eigen::Matrix3d::Identity();
    Eigen::Vector3d k = omega / theta;
    Eigen::Matrix3d K = skew(k);
    return Eigen::Matrix3d::Identity()
         + std::sin(theta) * K
         + (1 - std::cos(theta)) * (K * K);
}

// SE(3) exp
inline Eigen::Matrix4d se3_exp(const Eigen::Vector3d& omega,
                               const Eigen::Vector3d& v,
                               double dt) {
    Eigen::Matrix3d R = exp_so3(omega * dt);
    Eigen::Vector3d t = v * dt;
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    T.topLeftCorner<3,3>() = R;
    T.topRightCorner<3,1>() = t;
    return T;
}

// Minimal preprocessing: stride downsample to Eigen matrices
void preprocess_point_cloud(
    const std::vector<LidarPoint>& raw_points,
    Eigen::MatrixXd &out_pts,     // 3 x N
    Eigen::VectorXd &out_vels,    // N
    int downsample_factor = 1);
