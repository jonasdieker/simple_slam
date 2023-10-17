#include "simple_slam/data_loader.hpp"
#include "simple_slam/visual_odometry.hpp"

#include <iostream>
#include <string>

int main() {
  std::string dir = "data/00/";
  VisualOdometry vo{dir};
  vo.run_vo_pipeline();
}