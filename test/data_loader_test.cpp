#include "simple_slam/data_loader.hpp"
#include <gtest/gtest.h>
#include <filesystem>

TEST(DataLoaderTest, Initialize)
{
    std::filesystem::path current_path{std::filesystem::current_path()};
    std::filesystem::path relativ_path{"../../data/00/"};
    std::filesystem::path total_path = current_path / relativ_path;
    DataLoader data_loader{total_path};
    EXPECT_EQ(data_loader.get_cam_matrix().rows, 3);
    EXPECT_EQ(data_loader.get_cam_matrix().cols, 4);
}