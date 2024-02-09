#!/bin/bash

current_dir=$(pwd)
if [[ "$current_dir" =~ bfp_generation/build$ ]]; then
    cp ../bfp_gps.csv ../../ws/src/px4_ros_com//src/examples/offboard/
elif [[ "$current_dir" =~ bfp_generation$ ]]; then
    cp bfp_gps.csv ../ws/src/px4_ros_com//src/examples/offboard/
fi
