cmake_minimum_required(VERSION 3.22)
project(h1_2_controller)

# Find dependencies
find_package(Boost REQUIRED COMPONENTS program_options)
find_package(yaml-cpp REQUIRED)

# Set ONNX Runtime path based on architecture
if(CMAKE_SYSTEM_PROCESSOR STREQUAL "x86_64")
    set(ONNXRUNTIME_DIR "${PROJECT_SOURCE_DIR}/../../thirdparty/onnxruntime-linux-x64-1.22.0")
elseif(CMAKE_SYSTEM_PROCESSOR STREQUAL "aarch64")
    set(ONNXRUNTIME_DIR "${PROJECT_SOURCE_DIR}/../../thirdparty/onnxruntime-linux-aarch64-1.22.0")
endif()

# Include directories
include_directories(
    include                                  # project headers
    ${PROJECT_SOURCE_DIR}/../../include      # project-wide headers like FSM/
    /opt/unitree_robotics/include             # unitree + CycloneDDS headers
    ${ONNXRUNTIME_DIR}/include
    /usr/include
    /usr/local/include/ddscxx.BAK
    /usr/local/include/iceoryx/v2.0.2
)

# Library directories
link_directories(
    /opt/unitree_robotics/lib                 # contains libunitree_sdk2.a
    ${ONNXRUNTIME_DIR}/lib
)

# Source files
file(GLOB_RECURSE ADD_SRC_LIST
    ${PROJECT_SOURCE_DIR}/src/*.cpp
)

# Build library
add_library(${PROJECT_NAME}_lib ${ADD_SRC_LIST})

# Link libraries to the library target
target_link_libraries(${PROJECT_NAME}_lib
    unitree_sdk2        # static library in /opt/unitree_robotics/lib
    ddsc
    ddscxx
    pthread
    rt
    fmt
    Boost::program_options
    yaml-cpp
    ${ONNXRUNTIME_DIR}/lib/libonnxruntime.so.1.22.0
)

# Build executable
add_executable(h1_2_ctrl main.cpp)
target_link_libraries(h1_2_ctrl ${PROJECT_NAME}_lib)