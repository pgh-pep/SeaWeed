cmake_minimum_required(VERSION 3.8)
project(seaweed_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
    "action/MotionPlanner.action"
    DEPENDENCIES
    geometry_msgs
    nav_msgs
)

ament_auto_package()
