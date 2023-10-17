add_executable(vis_data vis_data.cpp)
target_link_libraries(vis_data PRIVATE data_loader)

add_executable(run_vo run_vo.cpp)
target_link_libraries(run_vo PRIVATE visual_odometry)