add_library(data_loader data_loader.cpp)
target_link_libraries(data_loader PUBLIC cxx_setup ${OpenCV_LIBS} boost_system boost_filesystem)



# add_library(visual_odom visual_odom.cpp)
# target_link_libraries(main PUBLIC data_loader)