#!/usr/bin/env bash

# Record topics required to debug tests

OUTPUT_PATH="$(echo $1 | sed 's![^/]$!&/!')"
echo "Bag file will be saved in "$OUTPUT_PATH

rosbag record -o "$OUTPUT_PATH"mbf_test \
              -e "(.*)/search_markers(.*)" \
              -e "(.*)/map(.*)" \
