# Altera FPGA Drive-on-Chip Control Driver

[ROS Control](https://control.ros.org/rolling/index.html) hardware interface for [Altera Drive-on-Chip](https://github.com/altera-fpga/agilex-ed-drive-on-chip).

The hardware interface supports the following interfaces:

### Control Interfaces

* Position

### State Interfaces

* Position
* Velocity

The hardware interface will scan for Drive-on-Chip UIO devices during initialization and assign each device to a robot joint based on the robot description. An error will be thrown if there are not enough devices present or the robot configuration requires a state or control interface which is not supported.

 > **Note:** The hardware interface does not currently have a mechanism to map Drive-on-Chip devices to robot joints so there is no guarantee the same device will be mapped to the same joint each time the interface is initialized.

## Setup
This hardware interface supports Altera SoC FPGA devices which are capable of running the [Drive-on-Chip](https://github.com/altera-fpga/agilex-ed-drive-on-chip) example design. Select your board from the list below for detailed setup instructions.

* [Agilex™ 5 FPGA E-Series Modular Development Kit](docs/agilex5_mk_a5e065bb32aes1.md)


Once Linux is booting on your board check you have the Drive-on-Chip devices mapped successfully to the Hard Processor System (HPS):

```
ls /dev/uio*
```
For the 3x2 (6) axis design you should see 3x UIO devices listed.
```
/dev/uio0  /dev/uio1  /dev/uio2
```

### Docker

If using the hardware interface from a Docker container ensure the Drive-on-Chip devices are passed through to the container using the `--device` argument.

For example:

```
docker run -it --rm --network host --device /dev/uio0 --device /dev/uio1 --device /dev/uio2 altera-ros2:arm64
```

## Usage
To use the Drive-on-Chip hardware interface with your own robot you will need to update the `ros2_control` section of the robot description (`.urdf`).

For example:

```
<ros2_control name="my_robot" type="system">
    <hardware>
        <plugin>fpga_doc_control_driver/DoCSystem</plugin>
    </hardware>
</ros2_control>
```

 Ensure the robot joints do not define any state or control interfaces not supported by this hardware interface.

 Here is an example of a supported joint definition:

 ```
<ros2_control name="my_robot" type="system">
    <joint name="joint_a">
        <command_interface name="position"/>
        <state_interface name="position"/>
        <state_interface name="velocity"/>
    </joint>
</ros2_control>
```
