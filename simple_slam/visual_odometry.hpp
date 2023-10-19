#pragma once
#include <utility>
#include <vector>

#include "opencv2/opencv.hpp"

// Define a type alias for a map of features (key points)
using Features =
    std::pair<std::vector<cv::KeyPoint>, std::vector<cv::KeyPoint>>;

class VisualOdometry {
 public:
  explicit VisualOdometry(const cv::Mat &cam_matrix);
  int process_frames(const cv::Mat &frame2, const cv::Mat &frame1);
  Features get_features();
  cv::Mat get_relative_pose();

 private:
  Features get_matched_features(const cv::Mat &img1, const cv::Mat &img2);
  double get_scale(const cv::Mat prev_pose, const cv::Mat cur_pose);
  cv::Mat cam_matrix_{};
  Features features_{};
  cv::Mat E_{};
  cv::Mat T_{};
};