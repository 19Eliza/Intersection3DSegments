cmake_minimum_required(VERSION 3.22)
project(SegmentIntersection LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Eigen3 REQUIRED)

add_subdirectory(GeometryObjects)

add_executable(SegmentIntersection main.cpp)

target_link_libraries(SegmentIntersection
    PRIVATE GeometryObjectsLib
    Eigen3::Eigen
)
