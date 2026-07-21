# go2_deploy

Deployment code of RL policy on Unitree Go2 robot, using policies from [genesis_lr](https://github.com/lupinjia/genesis_lr). This framework is based on the [state machine example from Unitree Doc](https://support.unitree.com/home/zh/developer/LowLevel_Ctrl_Framework) and conducts neural network inference via [LibTorch](https://pytorch.org/).

## Platform

- [Nvidia Jetson Orin NX 100Tops](https://www.nvidia.com/en-us/autonomous-machines/embedded-systems/jetson-orin/)
- x86_64 PC

## Installation

The controller must be built natively for the target architecture. In particular, do not copy the
x86-64 LibTorch, SDK2 libraries, or `build/` directory to a Jetson. Build/install aarch64 versions on
the Jetson itself.

1. Install `yaml-cpp`:

   ```bash
   sudo apt update
   sudo apt install libyaml-cpp-dev
   ```

2. Build and install [unitree_sdk2](https://github.com/unitreerobotics/unitree_sdk2) on the target machine:

   ```bash
   apt-get install -y cmake g++ build-essential libyaml-cpp-dev libeigen3-dev libboost-all-dev libspdlog-dev libfmt-dev
   git clone https://github.com/unitreerobotics/unitree_sdk2.git
   cmake -S unitree_sdk2 -B unitree_sdk2/build \
     -DCMAKE_BUILD_TYPE=Release \
     -DCMAKE_INSTALL_PREFIX=/opt/unitree_robotics
   cmake --build unitree_sdk2/build -j$(nproc)
   sudo cmake --install unitree_sdk2/build
   ```

3. Build or install LibTorch for the target machine. On a Jetson, it must be an aarch64 build compatible
   with the JetPack CUDA version installed on that Jetson. Record these machine-specific paths:

   - `PACT_TORCH_ROOT`: directory containing `share/cmake/Torch/TorchConfig.cmake` and `lib/`
   - `PACT_CUDA_ROOT`: CUDA toolkit directory containing `bin/nvcc`, normally `/usr/local/cuda`
   - `PACT_UNITREE_SDK2_ROOT`: SDK2 installation prefix, normally `/opt/unitree_robotics`

   Confirm that the libraries match the machine before configuring:

   ```bash
   uname -m
   file /path/to/libtorch/lib/libtorch.so
   file /opt/unitree_robotics/lib/libddsc.so
   /usr/local/cuda/bin/nvcc --version
   ```

   On the Jetson, `uname -m` and the libraries should report `aarch64`/`ARM aarch64`, not `x86-64`.

4. Configure and build the deployment code with paths for that machine:

   ```bash
   cmake -S . -B build \
     -DCMAKE_BUILD_TYPE=Release \
     -DPACT_CUDA_ROOT=/usr/local/cuda \
     -DPACT_TORCH_ROOT=/path/to/aarch64/libtorch \
     -DPACT_UNITREE_SDK2_ROOT=/opt/unitree_robotics
   cmake --build build -j$(nproc)
   ```

   The workstation can use different values. When LibTorch is located at the repository-adjacent
   `../libtorch` and SDK2 is installed under `/opt/unitree_robotics`, only `PACT_CUDA_ROOT` needs to be
   supplied. CMake stores these values in `build/CMakeCache.txt`; use a separate build directory per
   machine or delete the build directory when moving the source tree between machines.

5. Clone unitree_mujoco and compile (for simulation in mujoco)
   
   1. install mujoco
      ```bash
      sudo apt install libglfw3-dev libxinerama-dev libxcursor-dev libxi-dev

      git clone https://github.com/google-deepmind/mujoco.git
      mkdir build && cd build
      cmake ..
      make -j4
      sudo make install

      sudo apt install libyaml-cpp-dev
      ```
   2. install unitree_mujoco
      ```bash
      git clone https://github.com/lupinjia/unitree_mujoco.git
      cd unitree_mujoco/simulate
      mkdir build && cd build
      cmake ..
      make -j4
      ```

## Sim2Sim

1. Start the simulation
   ```bash
   # Start mujoco simulation
   cd unitree_mujoco/simulate/build
   ./unitree_mujoco
   ```

2. Start the controller
   ```bash
   cd go2_deploy/build
   ./go2_deploy
   ```
3. Play with the joystick
   - Common state machine logic:
      - L1 + R1 -> Sit
      - L1 + R2 -> Stand
      - L1 + A -> Ctrl
      - L1 + B -> Stop

## Sim2Real

Connect the Jetson's Ethernet port to the Go2 expansion dock and identify the wired interface:

```bash
ip -brief link
ip -brief address
```

Before running the controller, use the read-only DDS test. It subscribes to `rt/lowstate`; it does not
publish motor commands or disable the Go2 motion service:

```bash
./scripts/test_dds_connectivity.sh <ethernet-interface> 5
# Example:
./scripts/test_dds_connectivity.sh eth0 5
```

A working connection prints `PASS` and the number of low-state messages received. If it reports no
messages, verify the interface name, physical connection, interface IP/subnet, firewall, and SDK2 build.

After DDS passes, start the controller from the repository root:

```bash
./build/go2_deploy <ethernet-interface>
```

The real-robot executable disables the `mcf` service before starting low-level control. Place the robot
on a support fixture, keep the wireless controller and emergency stop available, and validate the state
transitions before enabling PACT control.

## Demo

| Controller Type | GIF | Training Code |
|--- | --- | --- |
|  Walk These Ways  | ![](https://raw.githubusercontent.com/lupinjia/demo_imgs/refs/heads/master/wtw_demo.gif) | [genesis_lr/go2_wtw](https://github.com/lupinjia/genesis_lr/tree/main/legged_gym/envs/go2/go2_wtw) |
| Teacher-Student | ![](https://raw.githubusercontent.com/lupinjia/demo_imgs/refs/heads/master/ts_demo.gif) | [genesis_lr/go2_ts](https://github.com/lupinjia/genesis_lr/tree/main/legged_gym/envs/go2/go2_ts) |

## Tips

1. To customize your own RL inference code, you need to create a new class in `include/user_controller.hpp` inheriting `BasicUserController`. An example RLController has been provided, which implements NN inference using basic apis of libtorch and double ended queue.
2. It's recommended to first simulate in mujoco and then deploy to real robot to avoid potential collapse.

## Acknowledgement

- [unitree_sdk2](https://github.com/unitreerobotics/unitree_sdk2)
- [unitree_mujoco](https://github.com/unitreerobotics/unitree_mujoco/tree/main)
