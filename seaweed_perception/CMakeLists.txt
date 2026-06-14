cmake_minimum_required(VERSION 3.12)
project(seaweed_perception)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_POLICY_DEFAULT_CMP0074 NEW)
set(CMAKE_POLICY_DEFAULT_CMP0144 NEW)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

find_package(PCL REQUIRED)

# utils
ament_auto_add_library(perception_utils SHARED
    ${PROJECT_NAME}/perception_utils.cpp
)

# Cpp Nodes
ament_auto_add_executable(
    euclidian_clustering_node
    ${PROJECT_NAME}/euclidian_clustering_node.cpp
    ${PROJECT_NAME}/perception_utils.cpp
)

target_link_libraries(euclidian_clustering_node ${PCL_LIBRARIES})

ament_auto_add_executable(
    bbox_projection_node
    ${PROJECT_NAME}/bbox_projection_node.cpp
    ${PROJECT_NAME}/perception_utils.cpp
)

install(TARGETS
    euclidian_clustering_node
    bbox_projection_node
    DESTINATION lib/${PROJECT_NAME}
)

ament_python_install_package(${PROJECT_NAME})

# Python Nodes:
install(PROGRAMS
    ${PROJECT_NAME}/rtcm_serial.py
    # ${PROJECT_NAME}/my_node.py
    ${PROJECT_NAME}/yolo_node.py
    DESTINATION lib/${PROJECT_NAME}
)

ament_auto_package(
    USE_SCOPED_HEADER_INSTALL_DIR
    INSTALL_TO_SHARE launch config rviz models
)
