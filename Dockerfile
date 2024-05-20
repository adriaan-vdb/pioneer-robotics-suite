FROM osrf/ros:humble-desktop AS base
# FROM althack/ros2:humble-dev AS base

SHELL ["/bin/bash", "-c"]

ENV DEBIAN_FRONTEND=noninteractive

RUN useradd -ms /bin/bash ros

RUN apt-get update \
    && apt-get -y install \
    wget ros-humble-navigation2 ros-humble-rviz2 ros-humble-slam-toolbox ros-humble-nav2-bringup ros-humble-teleop-twist-joy ros-humble-depthai-ros-driver\
    ros-humble-robot-localization ros-humble-joy-linux ros-humble-cyclonedds ros-humble-phidgets-spatial ros-humble-sick-scan-xd ros-humble-pointcloud-to-laserscan \
    doxygen libxcb-xinerama0 ros-humble-xacro vim ros-humble-tf-transformations \
    # libxcb* \
    && apt-get autoremove -y \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*

RUN wget https://bootstrap.pypa.io/get-pip.py && python3 get-pip.py && pip3 install setuptools==58.2.0 && rm get-pip.py

# RUN pip3 install yolov8

RUN pip3 install numpy && pip3 install pillow && pip3 install easyocr && pip3 install transforms3d

# RUN  "source /path/to/ros_entrypoint.sh"


RUN pip3 install opencv-python

# +++++++++++++++++++++++++++++++++++++++++++++++++++
FROM base as common
#ENV WS=nUWAy_ros2_ws
ENV WS=unity_ros2_ws
ENV WORKSPACE=/workspaces/$WS
WORKDIR /workspaces

RUN git clone https://github.com/reedhedges/AriaCoda.git
RUN cd AriaCoda && make && make install

COPY --chown=ros:ros --chmod=700 . ${WORKSPACE}
RUN chown -R ros ${WORKSPACE}
RUN gpasswd --add ros dialout

# COPY --chown=ros default.rviz /home/ros/.rviz2/default.rviz

COPY .EasyOCR /home/ros/.EasyOCR

RUN chmod 777 /home/ros/.EasyOCR/

COPY --chown=ros ros_entrypoint.sh /ros_entrypoint.sh
RUN chmod +x  /ros_entrypoint.sh
ENTRYPOINT ["/ros_entrypoint.sh"]

WORKDIR ${WORKSPACE}
USER ros

# RUN ssh-keyscan -t rsa github.com >> ~/.ssh/known_hosts
RUN ./build.sh