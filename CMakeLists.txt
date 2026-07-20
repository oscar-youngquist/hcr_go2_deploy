cmake_minimum_required(VERSION 3.16 FATAL_ERROR)

# Keep Torch/Caffe2 on a single CUDA toolkit version. On this machine
# /usr/local/cuda points to 12.9, but the LibTorch package was set up against 12.4.
set(CUDA_TOOLKIT_ROOT_DIR "/usr/local/cuda-12.4")
set(CUDAToolkit_ROOT "/usr/local/cuda-12.4")
set(CMAKE_CUDA_COMPILER "/usr/local/cuda-12.4/bin/nvcc")

project(go2_deploy LANGUAGES CXX)

SET(CMAKE_CXX_STANDARD 17)
list(APPEND CMAKE_PREFIX_PATH "/opt/unitree_robotics/lib/cmake")
find_package(unitree_sdk2 REQUIRED)

# set(CMAKE_PREFIX_PATH /home/lupinjia/libtorch)  # Change the path to your LibTorch

# Point CMake at the LibTorch install root
list(APPEND CMAKE_PREFIX_PATH "${PROJECT_SOURCE_DIR}/../libtorch")

message("-- CMAKE_PREFIX_PATH: ${CMAKE_PREFIX_PATH}")

# Optional: if auto-discovery still fails, use this instead:
set(Torch_DIR "${PROJECT_SOURCE_DIR}/../libtorch/share/cmake/Torch")

find_package(Torch REQUIRED)

include_directories(
    include 
    ${TORCH_INCLUDE_DIRS})

add_executable(go2_deploy 
src/main.cpp)

target_link_libraries(go2_deploy
    unitree_sdk2 
    yaml-cpp
    ${TORCH_LIBRARIES}
    )
