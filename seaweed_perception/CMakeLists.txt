cmake_minimum_required(VERSION 3.8)
project(seaweed_perception)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

cmake_policy(SET CMP0074 NEW)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

find_package(PCL REQUIRED)

# Cpp Nodes
ament_auto_add_executable(
    euclidian_clustering_node
    ${PROJECT_NAME}/euclidian_clustering_node.cpp
    ${PROJECT_NAME}/perception_utils.cpp
)

target_link_libraries(euclidian_clustering_node ${PCL_LIBRARIES})

ament_auto_add_executable(
    cluster_cache
    ${PROJECT_NAME}/cluster_cache.cpp
    ${PROJECT_NAME}/perception_utils.cpp
)

install(TARGETS
    euclidian_clustering_node
    cluster_cache
    DESTINATION lib/${PROJECT_NAME}
)

ament_python_install_package(${PROJECT_NAME})

# Python Nodes:
install(PROGRAMS
    ${PROJECT_NAME}/yolo_node.py
    DESTINATION lib/${PROJECT_NAME}
)

ament_auto_package(
    USE_SCOPED_HEADER_INSTALL_DIR
    INSTALL_TO_SHARE launch config rviz models
)
