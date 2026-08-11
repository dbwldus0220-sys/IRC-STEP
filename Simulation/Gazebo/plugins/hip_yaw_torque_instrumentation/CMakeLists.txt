cmake_minimum_required(VERSION 3.16)
project(step_hip_yaw_torque_instrumentation)

find_package(gz-plugin2 REQUIRED COMPONENTS register)
find_package(gz-sim8 REQUIRED)

add_library(step-hip-yaw-torque-instrumentation-system SHARED
  HipYawTorqueInstrumentation.cc
)
target_compile_features(step-hip-yaw-torque-instrumentation-system PRIVATE
  cxx_std_17
)
target_link_libraries(step-hip-yaw-torque-instrumentation-system PRIVATE
  gz-plugin2::gz-plugin2
  gz-sim8::gz-sim8
)
