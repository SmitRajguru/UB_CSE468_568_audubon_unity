#!/bin/bash

echo "Audubon Unity Script"

# download the latest audubon unity package
source /opt/ros/noetic/setup.bash

# check if the catkin_ws folder exists
if [ ! -d "/home/cse4568/catkin_ws" ]; then
    echo "catkin_ws does not exist"
    # check if the catkin_noetic folder exists
    if [ ! -d "/mnt/c/catkin_noetic" ]; then
        echo "catkin_ws does not exist"
    else
        source /mnt/c/catkin_noetic/devel/setup.bash
    fi
else
    source /home/cse4568/catkin_ws/devel/setup.bash
fi

roscd audubon_unity
echo "Running in $(pwd)"
git clone https://github.com/SmitRajguru/UB_CSE468_568_audubon_unity.git
cp -R UB_CSE468_568_audubon_unity/* ./
rm -rf UB_CSE468_568_audubon_unity
chmod +x script/*
chmod +x src/*

