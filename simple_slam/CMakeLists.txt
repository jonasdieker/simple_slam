add_library(data_loader data_loader.cpp)
target_link_libraries(data_loader PUBLIC cxx_setup ${OpenCV_LIBS})

add_library(visual_odometry visual_odometry.cpp)
target_link_libraries(visual_odometry PUBLIC cxx_setup ${OpenCV_LIBS})

add_library(visualiser visualiser.cpp)
target_link_libraries(visualiser PUBLIC cxx_setup ${OpenCV_LIBS})