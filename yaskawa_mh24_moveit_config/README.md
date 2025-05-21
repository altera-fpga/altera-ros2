# Yaskawa MH24 MoveIt Config

Configuration for controlling Yaskawa Motoman MH24 (simulated) robot arm with MoveIt 2.

## Usage

### Simulated

To start a move group node using [mock hardware controllers](https://control.ros.org/rolling/doc/ros2_control/hardware_interface/doc/mock_components_userdoc.html) run the following command:

#### With RViz
    ros2 launch yaskawa_mh24_moveit_config demo.launch.py use_mock_hardware:=true

#### Headless
    ros2 launch yaskawa_mh24_moveit_config demo.launch.py use_mock_hardware:=true use_rviz:=false

### FPGA Drive Control (Drive-on-Chip)

 > See the documentation [HERE](../fpga_doc_control_driver/README.md) for more information on the Altera Drive-on-Chip hardware controller.

To start a move group node using the FPGA Drive-on-Chip hardware interface use the following command:

    ros2 launch yaskawa_mh24_moveit_config demo.launch.py use_rviz:=false

