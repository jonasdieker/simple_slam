#pragma once

#include <utility>
#include <vector>

#include "opencv2/opencv.hpp"

using Features =
    std::pair<std::vector<cv::KeyPoint>, std::vector<cv::KeyPoint>>;

class Visualiser {
public:
  Visualiser();
  void update_features(const cv::Mat &frame1,
                       const Features &features);
  void update_map();

private:
};