# Altera® ROS 2 Packages

A collection of ROS 2 packages to support Altera® FPGA products.

# Packages

## [Altera® Camera](altera_camera/README.md)
ROS camera driver which interfaces with [Altera® Video Frame Writer IP](https://docs.altera.com/r/docs/683329/25.1/video-and-vision-processing-suite-ip-user-guide/about-the-video-buffer-writer-ip).

## [FPGA Drive-on-Chip Control Driver](fpga_doc_control_driver/README.md)
[ROS Control](https://control.ros.org/rolling/index.html) hardware interface for [Altera® Drive-on-Chip](https://github.com/altera-fpga/agilex-ed-drive-on-chip).

## [Yaskawa MH24 MoveIt Config](yaskawa_mh24_moveit_config/README.md)
Configuration for controlling Yaskawa Motoman MH24 (simulated) robot arm with MoveIt 2.

## [Yaskawa MH24 Robot Description](yaskawa_mh24_resources_description/README.md)
Robot description (URDF) and 3D model files for Yaskawa Motoman MH24.

# Examples

## [MoveIt2 Demo Client](examples/moveit_demo_client/README.md)
A [MoveIt2](https://moveit.picknik.ai/main/index.html) C++ client which can plan and execute to random or provided pose goals. The client makes use of the [`MoveGroupInterface`](https://docs.ros.org/en/noetic/api/moveit_ros_planning_interface/html/classmoveit_1_1planning__interface_1_1MoveGroupInterface.html) class making it compatible with various robot arms.

## [xArm Block Demo](examples/xarm_block_demo/README.md)
A pick & place demo supporting the [UFactory Lite6](https://www.ufactory.cc/lite-6-collaborative-robot/) robot arm. The demo utilizes [MoveIt2](https://moveit.picknik.ai/main/index.html) to create patterns with blocks.

## [xArm Block Camera Demo](examples/xarm_block_camera_demo/README.md)
A camera based pick & place demo supporting the [UFactory Lite6](https://www.ufactory.cc/lite-6-collaborative-robot/) robot arm. The demo utilizes [ArUco marker detection](https://github.com/namo-robotics/aruco_markers) and [MoveIt2](https://moveit.picknik.ai/main/index.html) to detect, pick and place blocks.

# Usage

It is highly recommended to use the official [Docker image](https://hub.docker.com/r/alterafpga/ros2) which includes the ROS 2 packages and examples provided by this repository pre-installed and ready to use.

Pull the Docker image with the following command:

    docker pull alterafpga/ros2

Check the documentation in the individual package directories to understand how they can be used.

## Docker

### Headless (x86)

```
docker run -it --rm --network host alterafpga/ros2
```

### Headless (arm64)

```
docker run -it --rm --network host alterafpga/ros2
```

### HPS with Drive-on-Chip support

```
docker run -it --rm --network host --device /dev/uio0 --device /dev/uio1 --device /dev/uio2 alterafpga/ros2
```

For graphical output such as running `RViz` in a container it is recommended to use [Rocker](https://github.com/osrf/rocker). See the official documentation for [installation instructions](https://github.com/osrf/rocker#installation).

### Rocker (Intel x86)
```
rocker --x11 --devices /dev/dri --network host alterafpga/ros2
```

# Building

Before continuing ensure [Docker is installed](https://docs.docker.com/engine/install/) on your device.

Build the Docker image from the root of this repository with the following command (remove `buildx` from the command if not available):

    docker buildx build -f .docker/Dockerfile -t alterafpga/ros2 .

To [cross-compile](https://docs.docker.com/build/building/multi-platform/) for a different architecture (such as `arm64` to support the `Hard Processor System` on Altera FPGA SoC platforms) run the following commands:

    docker run --privileged --rm tonistiigi/binfmt --install all
    
    docker buildx build -f .docker/Dockerfile -t alterafpga/ros2 --platform linux/arm64 .
