#!/bin/bash

echo hi
source ~/.profile
eval "$(cat ~/.bashrc | tail -n +10)"


ros2 topic echo /robot_pose