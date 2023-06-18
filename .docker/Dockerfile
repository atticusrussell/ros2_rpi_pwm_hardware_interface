ARG ROS_DISTRO="humble"
FROM ros:$ROS_DISTRO
ARG BRANCH="humble"

ENV ROS_UNDERLAY /root/ws_rpi_pwm_hardware_interface/install
WORKDIR $ROS_UNDERLAY/../src

ADD https://raw.githubusercontent.com/atticusrussell/ros2_rpi_pwm_hardware_interface/$BRANCH/rpi_pwm_hardware_interface.repos rpi_pwm_hardware_interface.repos
RUN vcs import < rpi_pwm_hardware_interface.repos

RUN apt-get update && rosdep update && \
    rosdep install -iy --from-paths . && \
    rm -rf /var/lib/apt/lists/

RUN cd $ROS_UNDERLAY/.. && \
        . /opt/ros/${ROS_DISTRO}/setup.sh && \
        colcon build

# source entrypoint setup
RUN sed --in-place --expression \
      '$isource "$ROS_UNDERLAY/setup.bash"' \
      /ros_entrypoint.sh
