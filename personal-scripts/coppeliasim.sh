#!/bin/bash

echo hi
source ~/.profile
eval "$(cat ~/.bashrc | tail -n +10)"



cd ~/
cd CoppeliaSim_Edu_V4_6_0_rev18_Ubuntu22_04/
./coppeliaSim.sh
