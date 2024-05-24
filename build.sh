#!/usr/bin/bash

distro=humble

source /opt/ros/${distro}/setup.bash
colcon build --packages-select p3at_interface --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=1
source ./install/setup.bash
colcon build
source ./install/setup.bash