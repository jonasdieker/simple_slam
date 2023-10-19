#include "visualiser.hpp"

void Visualiser::update_features(const cv::Mat &frame1,
                                 const cv::Mat &frame2,
                                 const Features &features) {
  for (const auto &pt : features.first) {
    cv::circle(frame1, pt.pt, 2, cv::viz::Color::yellow(), -1);
  }
  cv::imshow("Features", frame1);
}
