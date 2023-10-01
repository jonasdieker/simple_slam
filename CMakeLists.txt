cmake_minimum_required(VERSION 3.16..3.24)
project(simple_slam VERSION 1
    DESCRIPTION "basic slam project"
    LANGUAGES CXX)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "" FORCE)
endif()

message(STATUS "CMAKE_BUILD_TYPE: ${CMAKE_BUILD_TYPE}")

# Create a pseudo-library to propagate the needed flags.
add_library(cxx_setup INTERFACE)
target_compile_options(cxx_setup INTERFACE -Wall -Wpedantic -Wextra)
target_compile_features(cxx_setup INTERFACE cxx_std_17)
target_include_directories(cxx_setup INTERFACE ${PROJECT_SOURCE_DIR})

# # Update the submodules here
# include(external/UpdateSubmodules.cmake)

# # Enable testing for this project
# include(CTest)

find_package(OpenCV REQUIRED)
find_package(Boost COMPONENTS system REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS})

add_definitions(
    -DBOOST_COROUTINE_NO_DEPRECATION_WARNING=1
    -DBOOST_ALL_DYN_LINK=1
    -DBOOST_ERROR_CODE_HEADER_ONLY=1
)

# Add subdirectories
# add_subdirectory(external)
add_subdirectory(${PROJECT_NAME})
add_subdirectory(examples)