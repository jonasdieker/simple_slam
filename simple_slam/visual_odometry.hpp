#pragma once
#include <optional>
#include <utility>
#include <vector>

#include "opencv2/opencv.hpp"

// Define a type alias for a map of features (key points)
using Features =
    std::pair<std::vector<cv::KeyPoint>, std::vector<cv::KeyPoint>>;

class VisualOdometry {
 public:
  explicit VisualOdometry(const cv::Mat &cam_matrix);
  std::optional<std::pair<Features, cv::Mat>> get_relative_pose(
      const cv::Mat &frame2, const cv::Mat &frame1);

 private:
  Features get_matched_features(const cv::Mat &img1, const cv::Mat &img2);
  double get_scale(const cv::Mat prev_pose, const cv::Mat cur_pose);
  cv::Mat cam_matrix_;
};