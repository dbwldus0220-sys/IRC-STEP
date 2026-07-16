cmake_minimum_required(VERSION 3.8)
project(robot_msgs)
add_compile_definitions(STEP_SAFETY_COMMAND_LOG)
add_compile_definitions(STEP_ROLL_RATE_LIMIT_SAFETY)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(dynamixel_sdk QUIET)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/MotionCommand.msg"
  "msg/LinePoint.msg"
  "msg/LinePointsArray.msg"
  "msg/LineResult.msg"
  "msg/FallResult.msg"
  "msg/MotionEnd.msg"
  "msg/BallResult.msg"
  "msg/HurdleResult.msg"
  "msg/HoopResult.msg"
)

add_executable(safety_control_node
  main.cpp
  callback.cpp
  dynamixel.cpp
  dynamixel_controller.cpp
  ../STEP_Dynamics.cpp
)

target_include_directories(safety_control_node PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}
  ${CMAKE_CURRENT_SOURCE_DIR}/..
)

ament_target_dependencies(safety_control_node
  rclcpp
  std_msgs
  sensor_msgs
  tf2
)

target_link_libraries(safety_control_node Eigen3::Eigen)

if(dynamixel_sdk_FOUND)
  ament_target_dependencies(safety_control_node dynamixel_sdk)
else()
  find_path(DYNAMIXEL_SDK_INCLUDE_DIR
    NAMES dynamixel_sdk/dynamixel_sdk.h
    HINTS /usr/local/include
  )
  find_library(DYNAMIXEL_SDK_LIBRARY
    NAMES dxl_x64_cpp
    HINTS /usr/local/lib
  )
  if(NOT DYNAMIXEL_SDK_INCLUDE_DIR OR NOT DYNAMIXEL_SDK_LIBRARY)
    message(FATAL_ERROR
      "dynamixel_sdk was not found as a ROS package or in /usr/local")
  endif()
  target_include_directories(safety_control_node PRIVATE
    ${DYNAMIXEL_SDK_INCLUDE_DIR}
  )
  target_link_libraries(safety_control_node
    ${DYNAMIXEL_SDK_LIBRARY}
  )
endif()

# This package defines and consumes its own robot_msgs interfaces.
rosidl_get_typesupport_target(robot_msgs_typesupport_target
  ${PROJECT_NAME} rosidl_typesupport_cpp)
target_link_libraries(safety_control_node
  ${robot_msgs_typesupport_target}
)

install(TARGETS safety_control_node
  DESTINATION lib/${PROJECT_NAME}
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
