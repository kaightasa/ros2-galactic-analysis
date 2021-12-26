#!/bin/zsh

export ROS_TRACE_DIR=~/ros2-galactic-analysis/evaluate
ros2 trace -s pubsub -k -u "ros2*"
