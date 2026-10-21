cmake_minimum_required(VERSION 3.10)
project(pcd_reader)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(PCL 1.12 REQUIRED COMPONENTS common io)
find_package(PCL 1.12 REQUIRED COMPONENTS common io filters)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(pcd_reader main.cpp)
target_link_libraries(pcd_reader ${PCL_LIBRARIES})