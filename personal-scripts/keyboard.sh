#!/bin/bash

echo hi
source ~/.profile
eval "$(cat ~/.bashrc | tail -n +10)"


ros2 run navigation keyboard