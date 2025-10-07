cmake_minimum_required(VERSION 3.22)
project(GeometryObjects LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_library(GeometryObjectsLib STATIC segment3d.cpp
    vector3d.cpp)

target_include_directories(GeometryObjectsLib PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})