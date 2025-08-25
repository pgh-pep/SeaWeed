cmake_minimum_required(VERSION 3.8)
project(seaweed_perception)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_python_install_package(${PROJECT_NAME})

# Python Nodes:
install(PROGRAMS
    # ${PROJECT_NAME}/my_node.py
    DESTINATION lib/${PROJECT_NAME}
)

# Cpp Nodes
ament_auto_add_executable(
    surface_scan_node
    ${PROJECT_NAME}/surface_scan_node.cpp
)

# Only need if have non-ROS dependencies:
# target_link_libraries(cpp_exe_name dependency)

install(TARGETS
    surface_scan_node
    DESTINATION lib/${PROJECT_NAME}
)

ament_auto_package(
    USE_SCOPED_HEADER_INSTALL_DIR
    INSTALL_TO_SHARE launch config models
)
