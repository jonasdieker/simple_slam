#include "visualiser.hpp"


Visualiser::Visualiser() {
  cv::namedWindow("image_viewer", cv::WINDOW_AUTOSIZE);
}

void Visualiser::update_features(const cv::Mat &frame1,
                                 const Features &features) {
  cv::Mat temp = frame1;
  for (const auto &pt : features.first) {
    cv::circle(temp, pt.pt, 2, cv::viz::Color::yellow(), -1);
  }
  cv::imshow("image_viewer", temp);
  cv::waitKey(0.1 * 1000); // milliseconds

}
