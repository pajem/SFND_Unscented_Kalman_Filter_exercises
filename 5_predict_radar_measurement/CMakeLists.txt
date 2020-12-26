cmake_minimum_required(VERSION 3.16)

set(CMAKE_CXX_STANDARD 14)

project(predict_radar_measurement)

find_package(Eigen3 REQUIRED)

add_executable(predict_radar_measurement
    main.cpp
    ukf.cpp
)
target_link_libraries (predict_radar_measurement Eigen3::Eigen)
