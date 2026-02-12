# Lx Camera SDK (Linux / ROS2 / C++)

This repository contains Linux SDK libraries and a ROS2 driver (`lx_camera_ros`).

## Contents

- `linux/SDK/include`: SDK headers
- `linux/SDK/lib`: SDK shared libraries
- `linux/Sample/C`: C/C++ sample programs
- `linux/Sample/ros2/lx_camera_node_ws`: ROS2 workspace

## Compatibility

- Recommended: Ubuntu 22.04 + ROS2 Humble
- Ubuntu 24.04 + ROS2 Jazzy may work, but is not officially verified in this repo.

## Why `linux/install.sh` Is Required

The ROS2 node dynamically loads:

- `/opt/Lanxin-MRDVS/lib/libLxCameraApi.so`

So SDK installation is mandatory before running the camera node.

`linux/install.sh` does all of the following:

1. Copies SDK headers/libs to `/opt/Lanxin-MRDVS`
2. Selects platform libs by architecture (`x86_64`, `aarch64`, `arm`)
3. Exports `/opt/Lanxin-MRDVS/lib` into `LD_LIBRARY_PATH`
4. Runs socket buffer setup

Run it once:

```bash
cd linux
sudo ./install.sh
```

Quick sanity check:

```bash
ls -l /opt/Lanxin-MRDVS/lib/libLxCameraApi.so
```

## Prerequisites

- ROS2 environment installed and sourced
- `colcon`, `ament_cmake`
- ROS2 dependencies (OpenCV, PCL, cv_bridge, etc.)

Install dependencies with rosdep:

```bash
cd linux/Sample/ros2/lx_camera_node_ws
rosdep install --from-paths src --ignore-src -r -y
```

## Build

```bash
cd linux/Sample/ros2/lx_camera_node_ws
colcon build
source install/setup.bash
```

For symlink install:

```bash
colcon build --symlink-install
```

If symlink build fails with
`existing path cannot be removed: Is a directory`, clean package artifacts and rebuild:

```bash
rm -rf build/lx_camera_ros install/lx_camera_ros log/latest_build/lx_camera_ros
colcon build --symlink-install --packages-select lx_camera_ros
```

## Run

### 1) Standard launch (auto camera start)

```bash
cd linux/Sample/ros2/lx_camera_node_ws
source install/setup.bash
ros2 launch lx_camera_ros lx_camera_ros.launch.py
```

This path uses `lx_camera_node` and auto runs lifecycle transitions internally.

### 2) Lifecycle launch (manual by default)

```bash
ros2 launch lx_camera_ros lx_camera_ros_lifecycle.launch.py
```

Default is manual lifecycle (`autostart:=false`), so camera does not stream until you transition:

```bash
ros2 lifecycle get /lx_camera_node/lx_camera_node
ros2 lifecycle set /lx_camera_node/lx_camera_node configure
ros2 lifecycle set /lx_camera_node/lx_camera_node activate
```

If you want lifecycle launch to auto transition:

```bash
ros2 launch lx_camera_ros lx_camera_ros_lifecycle.launch.py autostart:=true
```

### 3) Other app launches

```bash
ros2 launch lx_camera_ros obstacle.launch.py
ros2 launch lx_camera_ros obstacleV2.launch.py
ros2 launch lx_camera_ros pallet.launch.py
```

## Diagnostics

The driver publishes diagnostics to:

- `/diagnostics`

Check:

```bash
ros2 topic echo /diagnostics
```

Current diagnostics focus on stream health (device open/start/frame timeout).  
Frequency diagnostics are intentionally not enabled.

## Where to Edit Launch Parameters

Edit source launch files here:

```text
linux/Sample/ros2/lx_camera_node_ws/src/lx_camera_ros/src/launch/ubuntu22/
```

Then rebuild:

```bash
cd linux/Sample/ros2/lx_camera_node_ws
colcon build
```

## Common Error

If you see:

```text
Load lib failed: /opt/Lanxin-MRDVS/lib/libLxCameraApi.so: cannot open shared object file
```

It means SDK installation is missing or incorrect. Re-run:

```bash
cd linux
sudo ./install.sh
```
