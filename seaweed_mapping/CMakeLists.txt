cmake_minimum_required(VERSION 3.8)
project(seaweed_mapping)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

find_package(PCL REQUIRED)

# Cpp Nodes
ament_auto_add_executable(
    test_node
    ${PROJECT_NAME}/test_node.cpp
)

target_link_libraries(test_node ${PCL_LIBRARIES})

install(TARGETS
    test_node
    DESTINATION lib/${PROJECT_NAME}
)

ament_python_install_package(${PROJECT_NAME})

# Python Nodes:
install(PROGRAMS
    # ${PROJECT_NAME}/file.py
    DESTINATION lib/${PROJECT_NAME}
)

ament_auto_package(
    USE_SCOPED_HEADER_INSTALL_DIR
    INSTALL_TO_SHARE launch config rviz
)
