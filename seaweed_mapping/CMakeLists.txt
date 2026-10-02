cmake_minimum_required(VERSION 3.8)
project(seaweed_mapping)

set(CMAKE_POLICY_DEFAULT_CMP0074 NEW)
set(CMAKE_POLICY_DEFAULT_CMP0144 NEW)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()
find_package(PCL REQUIRED COMPONENTS common filters segmentation search)

ament_python_install_package(${PROJECT_NAME})

# Python Nodes:
install(PROGRAMS
    # ${PROJECT_NAME}/file.py
    DESTINATION lib/${PROJECT_NAME}
)

# Cpp Nodes
ament_auto_add_executable(
    point_cluster
    src/point_cluster.cpp
)

target_link_libraries(point_cluster ${PCL_LIBRARIES})

install(TARGETS
    # cpp_exe_name
    DESTINATION lib/${PROJECT_NAME}
)

ament_auto_package(
    USE_SCOPED_HEADER_INSTALL_DIR
    INSTALL_TO_SHARE launch config rviz
)
