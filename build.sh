#!/usr/bin/bash

distro=humble

source /opt/ros/${distro}/setup.bash
colcon build
source ./install/setup.bash