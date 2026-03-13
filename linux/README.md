# CameraSDK Linux

This repository contains the Lanxin MRDVS camera SDK for Linux and sample code.

## Directory Layout

- `SDK/`: SDK headers and prebuilt libraries.
- `Sample/C/`: C/C++ sample programs.
- `Sample/ros2/lx_camera_node_ws/`: ROS2 sample workspace (`lx_camera_ros`).

## Current Sample Scope (Simplified)

Obstacle and localization sample components were removed from `Sample` to keep the package minimal.

- C/C++ samples in active use:
  - `single_camera2`
  - `multi_cameras`
  - `application_pallet`
  - `frame_callback`
  - `arm_local_camera` (separate Makefile-based example)
- ROS2 launch/scripts in active use:
  - `lx_camera_ros.launch.py`
  - `lx_camera_ros_lifecycle.launch.py`
  - `pallet.launch.py`
  - `pallet.sh`
  - `rate.sh`

## Install SDK to `/opt` (Optional)

```bash
./install.sh
```

This installs headers/libs to `/opt/Lanxin-MRDVS` and updates `LD_LIBRARY_PATH`.
Use this only if you want system-wide SDK installation.

## Build C/C++ Samples

```bash
cmake -S Sample/C -B build_sample_c
cmake --build build_sample_c -j
```

## Build ROS2 Sample

```bash
cd Sample/ros2/lx_camera_node_ws
./build.sh
```

`lx_camera_ros` now links against the bundled `SDK/` in this repository, so `./install.sh` is not required for ROS2 build/run.

`build.sh` also tries to set socket buffer size automatically:
- default: `10MB`
- disable auto setup: `LX_AUTO_SET_SOCKET_BUFFER=0 ./build.sh`
- change size: `LX_SOCKET_BUFFER_MB=20 ./build.sh`

`build.sh` uses OpenMPI include hints when available:
- `/usr/lib/x86_64-linux-gnu/openmpi/include`

`libopenmpi-dev` is not a strict requirement for this package.
Install it only if your build environment fails with `mpi.h`-related errors.
