#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <Eigen/Dense>
#include <vector>
#include <xlsxwriter.h>
#include <string>
#include "farness_dicp_cpp/math_utils.hpp"
#include <deque>

class DopplerICPStitcher : public rclcpp::Node {
public:
    explicit DopplerICPStitcher(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
    ~DopplerICPStitcher();
private:
    void process_next_frame();
    void publish_outputs(const std::vector<LidarPoint> &frame);
        // --- Publish rate management ---
    rclcpp::TimerBase::SharedPtr publish_timer_;
    double publish_rate_;


    // pubs
    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr pub_lin_acc_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr pub_ang_vel_;

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_cloud_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_pose_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_path_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::TimerBase::SharedPtr timer_;
    // Subscribers
    rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr sub_lin_acc_;
    rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr sub_ang_vel_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_cloud_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_pose_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr sub_path_;

    // Callback functions
    void callbackLinAcc(const geometry_msgs::msg::Vector3::SharedPtr msg);
    void callbackAngVel(const geometry_msgs::msg::Vector3::SharedPtr msg);
    void callbackCloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
    void callbackPose(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
    void callbackPath(const nav_msgs::msg::Path::SharedPtr msg);
    void publish_timer_callback();   



    // params
    std::string frames_dir_;
    double velocity_threshold_{20.0};
    double frame_dt_{0.1};
    double voxel_size_{1.0};
    double lambda_doppler_start_;
    double lambda_doppler_end_;
    int lambda_schedule_iters_;
    int buffer_size_{5};
    int geometric_min_iters_;
    int doppler_min_iters_;
    bool reject_outliers_;
    double outlier_thresh_;
    int rejection_min_iters_;
    int max_iterations_{40};
    double tolerance_{1e-5};
    double geom_k_{0.1};
    double doppler_k_{0.25};
    double max_corr_dist_{0.4};
    int min_inliers_{10};


    // state
    std::vector<std::string> frame_files_;
    size_t current_frame_idx_{0};
    std::vector<LidarPoint> previous_frame_;
    Eigen::Matrix4d current_pose_{Eigen::Matrix4d::Identity()};
    std::vector<Eigen::Matrix4d> trajectory_;
    nav_msgs::msg::Path path_;

    // Ego-motion correction offset
    Eigen::Vector3d t_vl_;

    // Excel logging (libxlsxwriter)
    lxw_workbook  *workbook_{nullptr};
    lxw_worksheet *worksheet_{nullptr};
    int excel_row_{1};
    std::string excel_file_;

    std::deque<std::vector<LidarPoint>> last_n_frames_;
    const size_t max_last_frames_ = 5;

};
