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

## Install SDK to `/opt`

```bash
./install.sh
```

This installs headers/libs to `/opt/Lanxin-MRDVS` and updates `LD_LIBRARY_PATH`.

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

`build.sh` expects OpenMPI headers at:
- `/usr/lib/x86_64-linux-gnu/openmpi/include`

If missing, install `libopenmpi-dev` or adjust include paths in `build.sh`.
