cmake_minimum_required(VERSION 3.16)

set(CMAKE_CXX_STANDARD 14)

project(predict_mean_and_covariance)

find_package(Eigen3 REQUIRED)

add_executable(predict_mean_and_covariance
    main.cpp
    ukf.cpp
)
target_link_libraries (predict_mean_and_covariance Eigen3::Eigen)
