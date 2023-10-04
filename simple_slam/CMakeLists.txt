add_library(data_loader data_loader.cpp)
target_link_libraries(data_loader PUBLIC cxx_setup ${OpenCV_LIBS} boost_system boost_filesystem)

add_library(visual_odometry visual_odometry.cpp)
target_link_libraries(visual_odometry PUBLIC data_loader)