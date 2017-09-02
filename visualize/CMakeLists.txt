cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(visualizer)

find_package(PCL 1.2 REQUIRED PATHS /usr NO_DEFAULT_PATH)

# 出力先の設定
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ../)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (visualizer src/visualizer.cpp)
target_link_libraries (visualizer ${PCL_LIBRARIES})
add_executable (oneviz src/oneviz.cpp)
target_link_libraries (oneviz ${PCL_LIBRARIES})
