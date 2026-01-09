cmake_minimum_required(VERSION 3.8)
project(seaweed_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
ament_auto_find_build_dependencies()

rosidl_generate_interfaces(${PROJECT_NAME}

    # "path_to_msg/action/srv"
    "msg/Detection.msg"
    "msg/BoundingBox.msg"
    "msg/LabeledPose.msg"
    "msg/LabeledPoseArray.msg"

    DEPENDENCIES
    std_msgs
    geometry_msgs

    # sensor_msgs
    # ...
)

ament_auto_package()
