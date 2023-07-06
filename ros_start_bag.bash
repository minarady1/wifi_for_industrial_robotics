#!/usr/bin/env bash
source /opt/ros/humble/setup.bash
. ~/ros2_ws/install/setup.bash

ros2 bag play ~/bagfiles/crawler --loop
