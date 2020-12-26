cmake_minimum_required(VERSION 3.16)

set(CMAKE_CXX_STANDARD 14)

project(measurement_update)

find_package(Eigen3 REQUIRED)

add_executable(measurement_update
    main.cpp
    ukf.cpp
)
target_link_libraries (measurement_update Eigen3::Eigen)
