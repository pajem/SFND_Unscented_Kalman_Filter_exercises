cmake_minimum_required(VERSION 3.16)

set(CMAKE_CXX_STANDARD 14)

project(augmented_sigma_points)

find_package(Eigen3 REQUIRED)

add_executable(augmented_sigma_points
    main.cpp
    ukf.cpp
)
target_link_libraries (augmented_sigma_points Eigen3::Eigen)
