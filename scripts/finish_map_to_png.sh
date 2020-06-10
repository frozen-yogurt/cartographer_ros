#!/bin/bash
echo "finish trajectory 0"
rosservice call /finish_trajectory 0
echo "finish trajectory 1"
rosservice call /finish_trajectory 1
echo "name of output pbstream file"
read FILENAME
PBSTREAM=${HOME}/Downloads/$FILENAME.pbstream
echo "generate file: $PBSTREAM"
rosservice call /write_state $PBSTREAM
echo "generate image"
rosrun cartographer_ros cartographer_pbstream_to_ros_map -pbstream_filename $PBSTREAM -map_filestem /home/box/Downloads/map_data/$FILENAME