#include "visual_odometry.hpp"

VisualOdometry::VisualOdometry(const std::string &data_dir)
    : data_loader_(data_dir) {
  cam_matrix_ = data_loader_.get_cam_matrix();
}

std::vector<cv::Point2f> VisualOdometry::extract_features(const cv::Mat &img) {
  int max_corners = 1000;
  double quality_level = 0.01;
  double min_distance = 1.0;

  std::vector<cv::Point2f> corners;
  cv::goodFeaturesToTrack(
      img, corners, max_corners, quality_level, min_distance);
  return corners;
}

Features VisualOdometry::get_matched_features(const cv::Mat &img1,
                                              const cv::Mat &img2) {
  // if images are RGB -> Grayscale
  if (img1.channels() > 1) { cv::cvtColor(img1, img1, cv::COLOR_BGR2GRAY); }
  if (img2.channels() > 1) { cv::cvtColor(img2, img2, cv::COLOR_BGR2GRAY); }

  std::vector<cv::KeyPoint> keypts1, keypts2;
  cv::Mat descriptors1, descriptors2;

  cv::Ptr<cv::ORB> orb = cv::ORB::create();
  orb->detectAndCompute(img1, cv::Mat(), keypts1, descriptors1);
  orb->detectAndCompute(img2, cv::Mat(), keypts2, descriptors2);

  // match ORB descriptors
  cv::Ptr<cv::DescriptorMatcher> matcher =
      cv::DescriptorMatcher::create("BruteForce-Hamming");
  std::vector<cv::DMatch> matches;
  matcher->match(descriptors1, descriptors2, matches);

  // filter matches based on threshold distances
  double min_dist = 10000;
  double max_dist = 0;

  for (int i = 0; i < descriptors1.rows; ++i) {
    double dist = matches[i].distance;
    if (dist < min_dist) min_dist = dist;
    if (dist > max_dist) max_dist = dist;
  }

  std::vector<cv::DMatch> good_matches;
  for (int i = 0; i < descriptors1.rows; ++i) {
    if (matches[i].distance <= std::max(2 * min_dist, 20.0)) {
      good_matches.push_back(matches[i]);
    }
  }

  std::vector<cv::KeyPoint> kp1_matched;
  std::vector<cv::KeyPoint> kp2_matched;

  for (const auto &match : good_matches) {
    kp1_matched.push_back(keypts1[match.queryIdx]);
    kp2_matched.push_back(keypts2[match.trainIdx]);
  }

  return std::make_pair(kp1_matched, kp2_matched);
}

double VisualOdometry::get_scale(int index) {
  cv::Point3d prev_point = {pred_poses_[index - 1].at<double>(0, 3),
                            pred_poses_[index - 1].at<double>(1, 3),
                            pred_poses_[index - 1].at<double>(2, 3)};
  cv::Point3d curr_point = {pred_poses_[index].at<double>(0, 3),
                            pred_poses_[index].at<double>(1, 3),
                            pred_poses_[index].at<double>(2, 3)};

  double scale = cv::norm(curr_point - prev_point);
  return scale;
}

void VisualOdometry::run_vo_pipeline() {
  // rotation and translation
  cv::Mat R, t;

  // bottom row of homogeneous transformation mat
  // cv::Mat bottom_row = (cv::Mat_<double>(1, 4) << 0, 0, 0, 1);
  cv::Mat bottom_row = cv::Mat(1, 4, CV_64F, {0, 0, 0, 1});

  // essential matrix
  cv::Mat E;
  E.convertTo(E, CV_64F);

  // 3x4 tranformation
  cv::Mat T_;

  // 4x4 homogeneous transformation
  cv::Mat T;

  // start loop over image pairs
  int idx = 0;
  auto img_pair = data_loader_.get_image_pair(idx);
  while (img_pair.has_value()) {
    // extract value from optional
    auto img_data = img_pair.value();

    // get matched features
    Features features{get_matched_features(img_data.first, img_data.second)};

    // add features to map
    map_.push_back(features);

    // // compute essential matrix
    // E = cv::findEssentialMat(
    //     features.second, features.first, cam_matrix_, cv::RANSAC,
    //     0.999, 1.0);

    // // extract pose
    // int inlier_cnt =
    //     cv::recoverPose(E, features.second, features.first, cam_matrix_, R,
    //     t);

    // std::cout << inlier_cnt << std::endl;

    // // assemble homogeneous transformation matrix
    // cv::hconcat(R, t, T_);
    // cv::vconcat(T_, bottom_row, T);

    // // get next pair of images
    // img_pair = data_loader_.get_image_pair(++idx);

    break;
  }
}
