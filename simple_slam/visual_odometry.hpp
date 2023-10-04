#pragma once

#include "data_loader.hpp"

// Define a type alias for a map of features (key points)
using Features = std::pair<std::vector<cv::KeyPoint>, std::vector<cv::KeyPoint>>;
using Map = std::vector<Features>;


class VisualOdometry {
public:
    explicit VisualOdometry(const std::string &data_dir);
    void run_vo_pipeline();

private:
    std::vector<cv::Point2f> extract_features(const cv::Mat &img);
    Features get_matched_features(const cv::Mat &img1, const cv::Mat &img2);
    double get_scale(int index);

    // data
    DataLoader data_loader_;
    cv::Mat cam_matrix_;
    std::vector<cv::Mat> gt_poses_;
    std::vector<cv::Mat> pred_poses_;
    Map map_;
};