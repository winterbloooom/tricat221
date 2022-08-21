#!/bin/sh

# permission to sensors
echo "========== Connect Sensors =========="
sudo chmod 777 /dev/tty* && ls -l /dev/tty*

# rosbag setting
date=$(date "+%y%m%d")
time=$(date "+%H%M%S")
num=$(($(find $(rospack find tricat221)/data/rosbag/${date}-??????-auto-* 2>/dev/null | wc -l)+1))
file_num=$(printf "%02d" "$num")
file_name=${date}-${time}-auto-${file_num}

# launch scripts
echo "\n========== Run =========="
roslaunch tricat221 autonomous.launch do_record:=1 filename:=${file_name}