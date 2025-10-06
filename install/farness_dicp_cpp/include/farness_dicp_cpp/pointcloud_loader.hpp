#pragma once

#include <string>
#include <vector>
#include <filesystem>
#include <algorithm>
#include <Eigen/Dense>
#include "farness_dicp_cpp/math_utils.hpp"
#include <rclcpp/rclcpp.hpp>

// ---- Load one LiDAR frame (.csv or .bin) ----
std::vector<LidarPoint> load_frame(
    const std::string &filename,
    double velocity_thresh,
    double voxel_size = 1.0);  // default matches YAML

// ---- List all frame files in a directory ----
inline std::vector<std::string> list_frame_files(const std::string &dir) {
    std::vector<std::string> files;

    for (const auto &entry : std::filesystem::directory_iterator(dir)) {
        if (!std::filesystem::is_regular_file(entry.path())) continue;
        auto ext = entry.path().extension().string();
        if (ext == ".csv" || ext == ".bin")
            files.push_back(entry.path().string());
    }

    std::sort(files.begin(), files.end());

    if (files.empty()) {
        RCLCPP_WARN(rclcpp::get_logger("pointcloud_loader"),
                    "No .csv or .bin files found in directory: %s", dir.c_str());
    }

    return files;
}
