#include "visual_odometry.hpp"

VisualOdometry::VisualOdometry(const cv::Mat &cam_matrix)
    : cam_matrix_(cam_matrix) {}

Features VisualOdometry::get_matched_features(const cv::Mat &img1,
                                              const cv::Mat &img2) {
  std::vector<cv::KeyPoint> keypts1, keypts2;
  cv::Mat descriptors1, descriptors2;

  cv::Ptr<cv::ORB> orb = cv::ORB::create(5000);
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

double VisualOdometry::get_scale(const cv::Mat prev_pose,
                                 const cv::Mat cur_pose) {
  cv::Point3d prev_point = {prev_pose.at<double>(0, 3),
                            prev_pose.at<double>(1, 3),
                            prev_pose.at<double>(2, 3)};
  cv::Point3d cur_point = {cur_pose.at<double>(0, 3),
                           cur_pose.at<double>(1, 3),
                           cur_pose.at<double>(2, 3)};

  double scale = cv::norm(cur_point - prev_point);
  return scale;
}

int VisualOdometry::process_frames(const cv::Mat &frame2,
                                   const cv::Mat &frame1) {
  // if images are RGB -> Grayscale
  cv::Mat frame1_gc;
  cv::Mat frame2_gc;
  cv::cvtColor(frame1, frame1_gc, cv::COLOR_BGR2GRAY);
  cv::cvtColor(frame2, frame2_gc, cv::COLOR_BGR2GRAY);

  // get matched features
  features_ = get_matched_features(frame1_gc, frame2_gc);

  std::vector<cv::Point2f> pts1, pts2;

  for (std::size_t i = 1; i < features_.first.size(); ++i) {
    pts1.push_back(features_.first[i].pt);
    pts2.push_back(features_.second[i].pt);
  }

  // compute essential matrix
  cv::Mat E_mask;
  E_mask.convertTo(E_mask, CV_64F);
  E_ = cv::findEssentialMat(
      pts2, pts1, cam_matrix_, cv::RANSAC, 0.999, 1.0, E_mask);

  if (E_.empty()) {
    std::cerr << "Error: Unable to compute essential matrix. \n";
    return 0;
  }

  // extract pose
  cv::Mat R, t;
  cv::recoverPose(E_, pts2, pts1, cam_matrix_, R, t);

  // assemble homogeneous transformation matrix
  cv::Mat T;
  cv::Mat bottom_row = cv::Mat(1, 4, CV_64F, {0, 0, 0, 1});
  cv::hconcat(R, t, T);
  cv::vconcat(T, bottom_row, T_);
  return 1;
}

Features VisualOdometry::get_features() { return features_; }

cv::Mat VisualOdometry::get_relative_pose() { return T_; }
