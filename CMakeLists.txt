cmake_minimum_required(VERSION 3.4)
project(opti_track)

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH    ${PROJECT_SOURCE_DIR}/lib)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -O3 -Wall -Wextra")

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

find_package(OpenCV 3.3.1 REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

include_directories(include)

add_executable(${PROJECT_NAME}
    example/ex_kitti.cc
    src/config.cc
    src/parameter.cc
    src/camera.cc
    src/feature_tracker.cc
)

target_link_libraries(${PROJECT_NAME}
    ${OpenCV_LIBS}
)