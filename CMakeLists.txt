cmake_minimum_required(VERSION 3.10)
project(Deadeye VERSION 0.1 LANGUAGES CXX)

# Project-wide setup
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

add_subdirectory(deadeye/daemon)

# Build main target
add_executable(deadeye-template
    src/main.cpp
    src/custom_pipeline.cpp
    src/custom_target_data.cpp
)
set_target_properties(deadeye-template PROPERTIES
  OUTPUT_NAME custom_deadeyed
  EXPORT_NAME custom_deadeyed
)

target_compile_definitions(deadeye-template
  PUBLIC
  $<$<BOOL:${DEADEYE_CAMERA0_PIPELINE}>:DEADEYE_CAMERA0_PIPELINE=${DEADEYE_CAMERA0_PIPELINE}>
  $<$<BOOL:${DEADEYE_CAMERA1_PIPELINE}>:DEADEYE_CAMERA1_PIPELINE=${DEADEYE_CAMERA1_PIPELINE}>
  $<$<BOOL:${DEADEYE_CAMERA2_PIPELINE}>:DEADEYE_CAMERA2_PIPELINE=${DEADEYE_CAMERA2_PIPELINE}>
  $<$<BOOL:${DEADEYE_CAMERA3_PIPELINE}>:DEADEYE_CAMERA3_PIPELINE=${DEADEYE_CAMERA3_PIPELINE}>
  $<$<BOOL:${DEADEYE_CAMERA4_PIPELINE}>:DEADEYE_CAMERA4_PIPELINE=${DEADEYE_CAMERA4_PIPELINE}>
)


target_link_libraries(deadeye-template
    PRIVATE
    Deadeye::library
)

