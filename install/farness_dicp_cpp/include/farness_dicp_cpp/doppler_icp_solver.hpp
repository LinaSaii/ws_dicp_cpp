#pragma once

#include "farness_dicp_cpp/math_utils.hpp"
#include <Eigen/Dense>
#include <vector>

struct DopplerICPResult {
    Eigen::Matrix4d transform = Eigen::Matrix4d::Identity();
    int iterations = 0;
    bool converged = false;
};

class DopplerICPSolver {
public:
    DopplerICPResult solve(
        const std::vector<LidarPoint>& src_points,
        const std::vector<LidarPoint>& tgt_points,
        int max_iterations,
        double tolerance,
        double dt,
        double geom_k,
        double doppler_k,
        double max_corr_dist,
        int min_inliers,
        double lambda_start,
        double lambda_end,
        int lambda_schedule_iters,
        int geometric_min_iters,
        int doppler_min_iters,
        bool reject_outliers,
        double outlier_thresh,
        int rejection_min_iters);
};
