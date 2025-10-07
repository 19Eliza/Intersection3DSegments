cmake_minimum_required(VERSION 3.22)
project(Intersection3DSegments)

file(GLOB PROJECT_SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/*.cpp)

find_package(Eigen3 REQUIRED)

add_subdirectory(GeometryObjects)

add_executable(${PROJECT_NAME} ${PROJECT_SOURCES})

target_link_libraries(Intersection3DSegments PRIVATE GeometryObjectsLib)

set_target_properties(${PROJECT_NAME} PROPERTIES
    CXX_STANDARD 20
    CXX_STANDARD_REQUIRED YES
    CXX_EXTENSIONS NO
)

target_link_libraries(${PROJECT_NAME} PRIVATE Eigen3::Eigen)
