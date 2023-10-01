#pragma once

#include <iostream>
#include <vector>
#include <utility>

#include "opencv2/opencv.hpp"


class DataLoader{
public:
    explicit DataLoader(const std::string &data_dir);
    cv::Mat get_cam_matrix();
    std::pair<cv::Mat, cv::Mat> get_data(int index);

private:
    void read_cam_matrix(const std::string &calib_file_name);
    void read_ground_truth_poses(const std::string &gt_file_name);
    void load_img_paths(const std::string &img_dir);
    cv::Mat get_image(int index);
    cv::Mat get_gt_pose(int index);

    // data
    std::string data_dir_{};
    std::vector<cv::Mat> gt_poses_{};
    std::vector<std::string> img_paths_{};
    cv::Mat P_{};
};