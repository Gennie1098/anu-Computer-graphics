cmake_minimum_required(VERSION 3.10)
project(homework1)

set(CMAKE_BUILD_TYPE "Debug")
set(CMAKE_CXX_STANDARD 17)

find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR})

add_executable(task1 task1.cpp Triangle.cpp)

add_executable(task3 task3.cpp rasterizer.hpp rasterizer.cpp Triangle.hpp Triangle.cpp mesh.cpp)
target_link_libraries(task3 ${OpenCV_LIBRARIES})

add_executable(task4 task4.cpp rasterizer.hpp rasterizer.cpp Triangle.hpp Triangle.cpp mesh.cpp)
target_link_libraries(task4 ${OpenCV_LIBRARIES})