FROM althack/ros2:humble-dev AS base

SHELL ["/bin/bash", "-c"]

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update \
    && apt-get -y install \
    wget ros-humble-navigation2 ros-humble-slam-toolbox ros-humble-nav2-bringup ros-humble-teleop-twist-joy \
    ros-humble-joy-linux ros-humble-cyclonedds \
    && apt-get autoremove -y \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*

RUN wget https://bootstrap.pypa.io/get-pip.py && python3 get-pip.py && pip3 install setuptools==58.2.0 && rm get-pip.py

# +++++++++++++++++++++++++++++++++++++++++++++++++++
FROM base as common
#ENV WS=nUWAy_ros2_ws
ENV WS=unity_ros2_ws
ENV WORKSPACE=/workspaces/$WS
WORKDIR /workspaces

COPY --chown=ros:ros --chmod=700 . ${WORKSPACE}
RUN chown -R ros ${WORKSPACE}

COPY --chown=ros ros_entrypoint.sh /ros_entrypoint.sh
RUN chmod +x  /ros_entrypoint.sh
ENTRYPOINT ["/ros_entrypoint.sh"]

WORKDIR ${WORKSPACE}
USER ros
RUN ./build.sh
