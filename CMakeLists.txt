cmake_minimum_required(VERSION 3.16 FATAL_ERROR)

# Machine-specific dependency roots. Supply these with -D on each machine.
set(PACT_CUDA_ROOT "" CACHE PATH "CUDA toolkit root (for example /usr/local/cuda)")
set(PACT_TORCH_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../libtorch" CACHE PATH "LibTorch installation root")
set(PACT_UNITREE_SDK2_ROOT "/opt/unitree_robotics" CACHE PATH "unitree_sdk2 installation prefix")

if(PACT_CUDA_ROOT)
    set(CUDA_TOOLKIT_ROOT_DIR "${PACT_CUDA_ROOT}" CACHE PATH "" FORCE)
    set(CUDAToolkit_ROOT "${PACT_CUDA_ROOT}" CACHE PATH "" FORCE)
    set(CMAKE_CUDA_COMPILER "${PACT_CUDA_ROOT}/bin/nvcc" CACHE FILEPATH "" FORCE)
endif()

project(go2_deploy LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

list(PREPEND CMAKE_PREFIX_PATH
    "${PACT_UNITREE_SDK2_ROOT}"
    "${PACT_TORCH_ROOT}")

set(Torch_DIR "${PACT_TORCH_ROOT}/share/cmake/Torch" CACHE PATH "Directory containing TorchConfig.cmake")

find_package(unitree_sdk2 REQUIRED)
find_package(Torch REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(Eigen3 REQUIRED)

message(STATUS "Target processor: ${CMAKE_SYSTEM_PROCESSOR}")
message(STATUS "CUDA root: ${PACT_CUDA_ROOT}")
message(STATUS "Torch root: ${PACT_TORCH_ROOT}")
message(STATUS "unitree_sdk2 root: ${PACT_UNITREE_SDK2_ROOT}")

add_library(go2_system
    src/HardwareBridge.cpp)

target_include_directories(go2_system PUBLIC
    include
    ${TORCH_INCLUDE_DIRS})

target_link_libraries(go2_system
    unitree_sdk2
    yaml-cpp
    Eigen3::Eigen
    ${TORCH_LIBRARIES})

add_executable(go2_deploy
    src/main.cpp)

target_link_libraries(go2_deploy
    go2_system
)

# Generate a project-local launcher that selects the CycloneDDS libraries from
# the same unitree_sdk2 installation used at configure time. This avoids ROS
# environments overriding them through LD_LIBRARY_PATH.
configure_file(
    scripts/run_go2_deploy.sh.in
    "${CMAKE_CURRENT_BINARY_DIR}/run_go2_deploy.sh"
    @ONLY)

# Read-only DDS diagnostic: subscribes to rt/lowstate and never publishes commands.
add_executable(dds_connectivity_test
    tools/dds_connectivity_test.cpp)

target_link_libraries(dds_connectivity_test
    unitree_sdk2)


# This is the correct CMake command for this control stack on the jetson
# cmake -S . -B build \
#   -DCMAKE_BUILD_TYPE=Release \
#   -DPACT_CUDA_ROOT=/usr/local/cuda \
#   -DPACT_TORCH_ROOT=/home/unitree/.local/lib/python3.8/site-packages/torch \
#   -DPACT_UNITREE_SDK2_ROOT=/opt/unitree_robotics
