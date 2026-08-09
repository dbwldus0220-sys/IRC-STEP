cmake_minimum_required(VERSION 3.16)
project(step_constant_pose_initializer)

find_package(gz-plugin2 REQUIRED COMPONENTS register)
find_package(gz-sim8 REQUIRED)

add_library(step-constant-pose-initializer-system SHARED
  ConstantPoseInitializer.cc
)
target_compile_features(step-constant-pose-initializer-system PRIVATE cxx_std_17)
target_link_libraries(step-constant-pose-initializer-system PRIVATE
  gz-plugin2::gz-plugin2
  gz-sim8::gz-sim8
)
