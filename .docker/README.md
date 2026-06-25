# Altera ROS 2 Docker Images

Docker images to get up and running with [Altera ROS 2 packages and examples](https://github.com/altera-fpga/altera-ros2). This is a collection of ROS 2 packages to support Altera FPGA products.

# Usage

Read the [main documentation](https://github.com/altera-fpga/altera-ros2/blob/main/README.md) for more information on the ROS packages included with this image and how to use them.

### Headless

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
