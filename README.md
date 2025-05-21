# Altera ROS 2 Packages

A collection of ROS 2 packages to support Altera FPGA products.

# Packages

## [FPGA Drive-on-Chip Control Driver](fpga_doc_control_driver/README.md)
[ROS Control](https://control.ros.org/rolling/index.html) hardware interface for [Altera Drive-on-Chip](https://github.com/altera-fpga/agilex-ed-drive-on-chip).

## [Yaskawa MH24 MoveIt Config](yaskawa_mh24_moveit_config/README.md)
Configuration for controlling Yaskawa Motoman MH24 (simulated) robot arm with MoveIt 2.

## [Yaskawa MH24 Robot Description](yaskawa_mh24_resources_description/README.md)
Robot description (URDF) and 3D model files for Yaskawa Motoman MH24.

# Examples

## [MoveIt2 Demo Client](examples/moveit_demo_client/README.md)
A [MoveIt2](https://moveit.picknik.ai/main/index.html) C++ client which can plan and execute to random or provided pose goals. The client makes use of the [`MoveGroupInterface`](https://docs.ros.org/en/noetic/api/moveit_ros_planning_interface/html/classmoveit_1_1planning__interface_1_1MoveGroupInterface.html) class making it compatible with various robot arms.

## [xArm Block Demo](examples/xarm_block_demo/README.md)
A pick & place demo supporting the [UFactory Lite6](https://www.ufactory.cc/lite-6-collaborative-robot/) robot arm. The demo utilizes [MoveIt2](https://moveit.picknik.ai/main/index.html) to create patterns with blocks.


# Building

It is highly recommended to use Docker if you wish to run any of the examples.

## Docker Build
A Docker build file is provided for ease of deployment. The resulting Docker image includes the ROS 2 packages and examples provided by this repository pre-installed and ready to use.

Before continuing ensure [Docker is installed](https://docs.docker.com/engine/install/) on your device.

Build the Docker image from the root of this repository with the following command:

    docker buildx build -f .docker/Dockerfile -t altera-ros2 .

To [cross-compile](https://docs.docker.com/build/building/multi-platform/) for a different architecture (such as `arm64` to support the `Hard Processor System` on Altera FPGA SoC platforms) run the following commands:

    docker run --privileged --rm tonistiigi/binfmt --install all
    docker buildx build -f .docker/Dockerfile -t altera-ros2:arm64 --platform linux/arm64 .

## Native Build
To build the packages natively please ensure the correct version of [ROS 2 is installed](https://docs.ros.org/en/rolling/Installation.html) on your device based on the branch of this repository you have checked out.

1. Create a workspace directory if you don't yet have one

```
mkdir -p $HOME/ros_ws/src
cd $HOME/ros_ws/src
```

2a. Clone this repository into `$HOME/ros_ws/src`

2b. (Optional) Clone the [xArm ROS 2 repository](https://github.com/xArm-Developer/xarm_ros2) into your source directory if you need need UFACTORY Lite 6 support

```
git clone https://github.com/xArm-Developer/xarm_ros2.git --recursive -b $ROS_DISTRO
```

3. Install dependencies

```
rosdep update
rosdep install -y \
    --from-paths . \
    --ignore-src
```

4. Build packages
```
cd $HOME/ros_ws
colcon build
```

4. Source the environment setup script
```
source install/setup.bash
```

# Usage

Check the documentation in the individual package directories to understand how they can be used.

## Docker

### Headless (x86)

```
docker run -it --rm --network host altera-ros2
```

### Headless (arm64)

```
docker run -it --rm --network host altera-ros2:arm64
```

### HPS with Drive-on-Chip support

```
docker run -it --rm --network host --device /dev/uio0 --device /dev/uio1 --device /dev/uio2 altera-ros2:arm64
```

For graphical output such as running `RViz` in a container it is recommended to use [Rocker](https://github.com/osrf/rocker). See the official documentation for [installation instructions](https://github.com/osrf/rocker#installation).

### Rocker (Intel x86)
```
rocker --x11 --devices /dev/dri --network host altera-ros2
```
