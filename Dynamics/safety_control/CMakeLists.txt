cmake_minimum_required(VERSION 3.8)
project(robot_msgs)
add_compile_definitions(STEP_SAFETY_COMMAND_LOG)
add_compile_definitions(STEP_ROLL_RATE_LIMIT_SAFETY)

option(STEP_DRY_RUN_NO_DXL
  "Run safety_control_node without Dynamixel hardware communication" OFF)
option(STEP_ROLL_SCALE_TEST
  "Enable command-specific roll scale testing" OFF)
option(STEP_COMMAND32_START_BLEND_TEST
  "Use a command_1-style start trajectory during command_32's first cycle" OFF)
option(STEP_COMMAND32_COMPENSATION_TEST
  "Ramp command_32 startup compensation over its first 135 frames" OFF)
option(STEP_COMMAND3_COMPENSATION_TEST
  "Disable command_3 angle compensation for isolation testing" OFF)
option(STEP_REAL_ROBOT_COMMAND_GATE
  "Allow only real-robot commands approved by the pre-check" OFF)
option(STEP_REAL_ROBOT_STARTUP_SAFE
  "Split Dynamixel startup preparation across commands 90 through 93" OFF)

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

if(STEP_DRY_RUN_NO_DXL)
  target_compile_definitions(safety_control_node PRIVATE STEP_DRY_RUN_NO_DXL=1)
  message(WARNING
    "STEP_DRY_RUN_NO_DXL is enabled: Dynamixel hardware communication is disabled")
endif()

if(STEP_ROLL_SCALE_TEST)
  target_compile_definitions(safety_control_node PRIVATE STEP_ROLL_SCALE_TEST=1)
  message(WARNING
    "STEP_ROLL_SCALE_TEST is enabled: command-specific roll scaling is active")
endif()

if(STEP_COMMAND32_START_BLEND_TEST)
  target_compile_definitions(
    safety_control_node PRIVATE STEP_COMMAND32_START_BLEND_TEST=1)
  message(WARNING
    "STEP_COMMAND32_START_BLEND_TEST is enabled: command_32 start trajectory test is active")
endif()

if(STEP_COMMAND32_COMPENSATION_TEST)
  target_compile_definitions(
    safety_control_node PRIVATE STEP_COMMAND32_COMPENSATION_TEST=1)
  message(WARNING
    "STEP_COMMAND32_COMPENSATION_TEST is enabled: command_32 startup compensation isolation is active")
endif()

if(STEP_COMMAND3_COMPENSATION_TEST)
  target_compile_definitions(
    safety_control_node PRIVATE STEP_COMMAND3_COMPENSATION_TEST=1)
  message(WARNING
    "STEP_COMMAND3_COMPENSATION_TEST is enabled: command_3 angle compensation isolation is active")
endif()

if(STEP_REAL_ROBOT_COMMAND_GATE)
  target_compile_definitions(
    safety_control_node PRIVATE STEP_REAL_ROBOT_COMMAND_GATE=1)
  message(WARNING
    "STEP_REAL_ROBOT_COMMAND_GATE is enabled: only approved normal commands are allowed")
endif()

if(STEP_REAL_ROBOT_STARTUP_SAFE)
  target_compile_definitions(
    safety_control_node PRIVATE STEP_REAL_ROBOT_STARTUP_SAFE=1)
  message(WARNING
    "STEP_REAL_ROBOT_STARTUP_SAFE is enabled: commands 90-93 are required before motion")
endif()

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
