# Copyright (C) 2025 Altera Corporation
# SPDX-License-Identifier: Apache-2.0


ARG ROS_DISTRO="jazzy"
ARG IMAGE=ros:$ROS_DISTRO-ros-base
ARG OVERLAY_WS=/opt/ros/altera

FROM $IMAGE

# Install dependencies
ARG ROS_PKGS="\
	ros-jazzy-rmw-cyclonedds-cpp \
"

ARG XARM_COMMIT="5385b12d179e5ef52d84eaf0d11d93ea6a7e6cc3"

ARG OVERLAY_WS
WORKDIR $OVERLAY_WS

COPY yaskawa_mh24_resources_description/CMakeLists.txt $OVERLAY_WS/yaskawa_mh24_resources_description/
COPY yaskawa_mh24_resources_description/package.xml $OVERLAY_WS/yaskawa_mh24_resources_description/
COPY yaskawa_mh24_resources_description/meshes/mh24/collision/*.stl $OVERLAY_WS/yaskawa_mh24_resources_description/meshes/mh24/collision/
COPY yaskawa_mh24_resources_description/meshes/mh24/visual/*.dae $OVERLAY_WS/yaskawa_mh24_resources_description/meshes/mh24/visual/
COPY yaskawa_mh24_resources_description/urdf/*.xacro $OVERLAY_WS/yaskawa_mh24_resources_description/urdf/

COPY fpga_doc_control_driver/CMakeLists.txt $OVERLAY_WS/fpga_doc_control_driver/
COPY fpga_doc_control_driver/*.xml $OVERLAY_WS/fpga_doc_control_driver/
COPY fpga_doc_control_driver/src/*.cpp $OVERLAY_WS/fpga_doc_control_driver/src/
COPY fpga_doc_control_driver/src/include/*.hpp $OVERLAY_WS/fpga_doc_control_driver/src/include/
COPY fpga_doc_control_driver/src/include/*.h $OVERLAY_WS/fpga_doc_control_driver/src/include/

COPY yaskawa_mh24_moveit_config/CMakeLists.txt $OVERLAY_WS/yaskawa_mh24_moveit_config/
COPY yaskawa_mh24_moveit_config/package.xml $OVERLAY_WS/yaskawa_mh24_moveit_config/
COPY yaskawa_mh24_moveit_config/.setup_assistant $OVERLAY_WS/yaskawa_mh24_moveit_config/
COPY yaskawa_mh24_moveit_config/config/*.xacro $OVERLAY_WS/yaskawa_mh24_moveit_config/config/
COPY yaskawa_mh24_moveit_config/config/*.srdf $OVERLAY_WS/yaskawa_mh24_moveit_config/config/
COPY yaskawa_mh24_moveit_config/config/*.yaml $OVERLAY_WS/yaskawa_mh24_moveit_config/config/
COPY yaskawa_mh24_moveit_config/config/*.rviz $OVERLAY_WS/yaskawa_mh24_moveit_config/config/
COPY yaskawa_mh24_moveit_config/launch/*.py $OVERLAY_WS/yaskawa_mh24_moveit_config/launch/

COPY examples/moveit_demo_client/config/*.rviz $OVERLAY_WS/moveit_demo_client/config/
COPY examples/moveit_demo_client/launch/*.py $OVERLAY_WS/moveit_demo_client/launch/
COPY examples/moveit_demo_client/src/*.cpp $OVERLAY_WS/moveit_demo_client/src/
COPY examples/moveit_demo_client/CMakeLists.txt $OVERLAY_WS/moveit_demo_client/
COPY examples/moveit_demo_client/package.xml $OVERLAY_WS/moveit_demo_client/

COPY examples/xarm_block_demo/config/*.rviz $OVERLAY_WS/xarm_block_demo/config/
COPY examples/xarm_block_demo/launch/*.py $OVERLAY_WS/xarm_block_demo/launch/
COPY examples/xarm_block_demo/src/*.cpp $OVERLAY_WS/xarm_block_demo//src/
COPY examples/xarm_block_demo/CMakeLists.txt $OVERLAY_WS/xarm_block_demo/
COPY examples/xarm_block_demo/package.xml $OVERLAY_WS/xarm_block_demo/

#################  UFACTORY  #################
RUN git clone https://github.com/xArm-Developer/xarm_ros2.git --recursive -b $ROS_DISTRO

# Patch
COPY .patch/xarm_ros2/*.patch $OVERLAY_WS/xarm_ros2/

WORKDIR $OVERLAY_WS/xarm_ros2
RUN git checkout $XARM_COMMIT && git apply *.patch

RUN sed -i '/#include \"hardware_interface\/visibility_control.h\"/d' xarm_controller/include/xarm_controller/hardware/uf_robot_system_hardware.h

# Enable services
RUN sed -i 's/open_lite6_gripper: false/open_lite6_gripper: true/g' xarm_api/config/xarm_params.yaml
RUN sed -i 's/close_lite6_gripper: false/close_lite6_gripper: true/g' xarm_api/config/xarm_params.yaml
RUN sed -i 's/stop_lite6_gripper: false/stop_lite6_gripper: true/g' xarm_api/config/xarm_params.yaml

RUN mv \
    uf_ros_lib \
    xarm_api \
    xarm_controller \
    xarm_description \
    xarm_gazebo \
    xarm_moveit_config \
    xarm_msgs \
    xarm_sdk \
    ..
###############################################

WORKDIR $OVERLAY_WS

# Cleanup
RUN rm -rf xarm_ros2

RUN . /opt/ros/$ROS_DISTRO/setup.sh && \
    apt update -q && \
    rosdep install -y \
      --from-paths \
        . \
      --ignore-src && \
    apt install -y --no-install-recommends ${ROS_PKGS} && \
    rm -rf /var/lib/apt/lists/*

# Build overlay source
ARG OVERLAY_MIXINS="release"
RUN . /opt/ros/$ROS_DISTRO/setup.sh && \
    colcon build \
      --mixin $OVERLAY_MIXINS

# Source entrypoint setup
ENV OVERLAY_WS=$OVERLAY_WS
RUN sed --in-place --expression \
      '$isource "$OVERLAY_WS/install/setup.bash"' \
      /ros_entrypoint.sh


ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

CMD ["bash"]
