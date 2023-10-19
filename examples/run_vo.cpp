#include "simple_slam/data_loader.hpp"
#include "simple_slam/visual_odometry.hpp"
#include "simple_slam/visualiser.hpp"

#include <iostream>
#include <string>
#include <typeinfo>

int main() {
  const std::string dir = "data/00/";
  DataLoader data_loader{dir};
  cv::Mat cam_matrix = data_loader.get_cam_matrix();
  VisualOdometry visual_odometry{cam_matrix};
  Visualiser visualiser{};

  int idx = 0;
  auto image_pair = data_loader.get_image_pair(idx);

  while (image_pair.has_value()) {
    // get relative pose
    cv::Mat frame1 = image_pair.value().first;
    cv::Mat frame2 = image_pair.value().second;

    std::cout << "test_before_vo" << std::endl;

    auto features_pose = visual_odometry.get_relative_pose(frame2, frame1);

    std::cout << "test" << std::endl;

    if (!features_pose.has_value()) {
      std::cout << "no value returned!" << std::endl;
      return 1;
    }

    cv::Mat relative_pose = features_pose.value().second;
    Features features = features_pose.value().first;

    std::cout << typeid(features).name() << std::endl;

    // std::cout << relative_pose << std::endl;

    // // update visualisation
    // visualiser.update_features(frame1, frame2, features);
    // // visualiser.update_map(relative_pose)

    // // update data loader
    // data_loader.get_image_pair(++idx);

    // break;
  }

  return 0;
}