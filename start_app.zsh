#!/bin/zsh

caret_pubsub

ldd ./build/pubsub/talker | grep rclcpp

#ros2 launch ~/ros2-galactic-analysis/src/launch/pubsub_sub1_launch.py
