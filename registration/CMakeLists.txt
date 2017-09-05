cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(icp)

# if(POLICY CMP0037)
#   cmake_policy(SET CMP0037 NEW)
# endif()

find_package(PCL 1.2 REQUIRED PATHS /usr NO_DEFAULT_PATH)

# 出力先の設定
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ../)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (icp src/icp.cpp)
target_link_libraries (icp ${PCL_LIBRARIES})
add_executable (svd_scale src/svd_scale.cpp)
target_link_libraries (svd_scale ${PCL_LIBRARIES})
add_executable (icp_svd_scale src/icp_svd_scale.cpp)
target_link_libraries (icp_svd_scale ${PCL_LIBRARIES})
