# Altera® Video Frame Writer Camera Driver

A ROS 2 camera driver which interfaces with [Altera® Video Frame Writer IP](https://docs.altera.com/r/docs/683329/25.1/video-and-vision-processing-suite-ip-user-guide/about-the-video-buffer-writer-ip).

## Setup
This camera interface supports Altera® SoC FPGA devices which are capable of running the [Altera® Robotics](https://github.com/altera-fpga/agilex-ed-robotics) example designs.

In these designs, [Altera® Video Frame Writers](https://docs.altera.com/r/docs/683329/25.1/video-and-vision-processing-suite-ip-user-guide/about-the-video-buffer-writer-ip) are mapped as UIO devices.

```
ls /dev/uio*

/dev/uio0  /dev/uio1
```

### Docker

If using the camera interface from a Docker container ensure the Video Frame Writer devices are passed through to the container using the `--device` argument.

For example:

```
docker run -it --rm --network host --device /dev/uio0 alterafpga/ros2
```

## Usage
Publish images, using default parameters:

    ros2 launch altera_camera altera_camera.launch.py

### Parameters

* `fw_device` - `string`, default: `"/dev/uio0"`

    The UIO device associated with the Altera(R) Video Frame Writer.

* `publish_rate` - `integer`, default: `30`

    Maximum number of images to publish per second.

* `camera_name` - `string`, default: `"altera-camera"`

    Camera name which is used for the published frame id and also to find the camera configuration file.
