cmake_minimum_required(VERSION 3.16)

set(CMAKE_CXX_STANDARD 14)

project(predict_sigma_points)

find_package(Eigen3 REQUIRED)

add_executable(predict_sigma_points
    main.cpp
    ukf.cpp
)
target_link_libraries (predict_sigma_points Eigen3::Eigen)
