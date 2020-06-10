#!/bin/bash
rosrun cartographer_ros cartographer_pbstream_to_ros_map -pbstream_filename ${HOME}/Downloads/$1.pbstream -map_filestem /home/box/Downloads/map_data/$1