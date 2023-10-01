#include "data_loader.hpp"
#include <filesystem>


DataLoader::DataLoader(const std::string &data_dir) : data_dir_(data_dir)
{
    read_cam_matrix(data_dir + "calib.txt");
    read_ground_truth_poses(data_dir + "00.txt");
    load_img_paths(data_dir + "image_2");
}

void DataLoader::read_cam_matrix(const std::string &calib_file_name)
{
    // open file
    std::ifstream calib{calib_file_name};

    // check if file is open
    if (!calib.is_open()) {
        std::cerr << "Unable to open calib file: " << calib_file_name << std::endl;
        return;
    }

    // read first line from file
    std::string line;
    std::getline(calib, line);

    std::stringstream ss(line);

    // discard "PO: "
    std::string s;
    ss >> s;

    std::vector<double> values(12);
    for (double &value : values) {
        ss >> value;
    }

    P_ = cv::Mat(values).reshape(0, 3);
    P_.convertTo(P_, CV_64F);

    std::cout << "Projection Matrix Size: " << P_.size() << std::endl;
    std::cout << "Projection Matrix: " << P_ << std::endl;
}

void DataLoader::read_ground_truth_poses(const std::string &gt_file_name)
{
    std::ifstream gt_file{gt_file_name};

    if (!gt_file.is_open()) {
        std::cerr << "Unable to open ground truth poses file: " << gt_file_name << std::endl;
        return;
    }

    std::string line;
    while (std::getline(gt_file, line)) {
        std::stringstream line_stream(line);
        std::vector<double> pose_values;
        double value;

        while (line_stream >> value) {
            pose_values.push_back(value);
        }

        //check if size if correct
        if (pose_values.size() != 12) {
            std::cerr << "Error: Incorrect number of values in line" << std::endl;
            return;
        }

        cv::Mat gt_pose = cv::Mat(pose_values).reshape(0, 3);
        gt_pose.convertTo(gt_pose, CV_64F);

        gt_poses_.push_back(gt_pose);
    }

}

void DataLoader::load_img_paths(const std::string &img_dir)
{
    try {
        for (const auto& entry : std::filesystem::directory_iterator(img_dir)) {
            img_paths_.push_back(entry.path());
        }
    } catch (const std::filesystem::filesystem_error& e) {
        std::cerr << "Error accessing the directory: " << e.what() << std::endl;
    }
}

cv::Mat DataLoader::get_image(int index)
{
    if (index >= static_cast<int>(img_paths_.size())) {
        std::cerr << "Index out of range: " << index << std::endl;
        return cv::Mat{};
    }
    cv::Mat img = cv::imread(img_paths_[index]);
    return img;
}

cv::Mat DataLoader::get_gt_pose(int index)
{
    if (index >= static_cast<int>(gt_poses_.size())) {
        std::cerr << "Index out of range: " << index << std::endl;
        return cv::Mat{};
    }
    return gt_poses_[index];
}

cv::Mat DataLoader::get_cam_matrix()
{
    return P_;
}

std::pair<cv::Mat, cv::Mat> DataLoader::get_data(int index)
{
    return {get_image(index), get_gt_pose(index)};
}