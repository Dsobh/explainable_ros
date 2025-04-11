cmake_minimum_required(VERSION 3.8)
project(explainable_ros_msgs)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/Question.srv"
  DEPENDENCIES
)

ament_package()
