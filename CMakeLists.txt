cmake_minimum_required (VERSION 2.8)
project (registration)

find_package (PCL 1.6 REQUIRED)
include_directories (${PCL_INCLUDE_DIRS})
link_directories (${PCL_LIBRIARY_DIRS})
add_definitions (${PCL_DEFINITIONS})

aux_source_directory (. SRC_LIST)
add_executable (${CMAKE_PROJECT_NAME} ${SRC_LIST})
target_link_libraries (${CMAKE_PROJECT_NAME} ${PCL_LIBRARIES})
