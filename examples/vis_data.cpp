#include "simple_slam/data_loader.hpp"

#include <string>

int main() {
    std::string dir = "data/00/";
    DataLoader data_loader{dir};
    auto data = data_loader.get_image_pair(0);
    if (data.has_value()) {
        auto img_pair = data.value();
        std::cout << img_pair.first.size() << ", " << img_pair.second.size() << std::endl;
    } else {
        std::cerr << "Invalid index or pair not available" << std::endl;
    }

    return 0;

}