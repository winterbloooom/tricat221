#!/bin/sh

# permission to sensors
echo "========== Connect Sensors =========="
sudo chmod 777 /dev/tty* && ls -l /dev/tty*
sudo chmod 777 /dev/video* && ls -l /dev/video*

# rosbag setting
date=$(date "+%y%m%d")
time=$(date "+%H%M%S")
num=$(($(find $(rospack find tricat221)/etc/rosbag/${date}-??????-docking-* 2>/dev/null | wc -l)+1))
file_num=$(printf "%02d" "$num")
file_name=${date}-${time}-docking-${file_num}

# launch scripts
echo "\n========== Run =========="
roslaunch tricat221 docking.launch filename:=${file_name}