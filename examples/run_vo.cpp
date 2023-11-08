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

    int vo_success = visual_odometry.process_frames(frame2, frame1);

    if (!vo_success) {
      std::cout << "no features/pose returned!" << std::endl;
      return 1;
    }

    cv::Mat relative_pose = visual_odometry.get_relative_pose();
    Features features = visual_odometry.get_features();

    std::cout << features.first.size() << " " << features.second.size()
              << std::endl;

    std::cout << relative_pose << std::endl;

    // update visualisation
    visualiser.update_features(frame1, features);
    // // visualiser.update_map(relative_pose)

    // update data loader
    auto image_pair = data_loader.get_image_pair(++idx);

    if (idx == 100) {
      break;
    }
  }

  return 0;
}