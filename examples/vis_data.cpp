#include "simple_slam/data_loader.hpp"

#include <string>

int main() {
    std::string dir = "/home/jonas/Downloads/00/";
    DataLoader data_loader{dir};
    auto data = data_loader.get_data(0);
    std::cout << data.first.size() << ", " << data.second.size() << std::endl;

    return 0;

}