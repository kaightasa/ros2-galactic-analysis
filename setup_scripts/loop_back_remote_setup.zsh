#!/bin/zsh

cd ~/ros2-galactic-analysis

# check argument
if [ $# -ne 1 ]; then
  echo "usage: loop_back_remote_setup.zsh <queue num>" 1>&2
  echo "ex: loop_back_remote_setup.zsh 10" 1>&2
  exit 1
fi

# set backup queue size
if [[ "$1" =~ ^[0-9]+$ ]]; then
  sed -i "28c\    publisher_ = this->create_publisher<time_interface::msg::Timestamp>(\"topic2\", $1);" src/loop_back/src/remote_member_function.cpp
else
  echo "use number for queue num. you set \"$1\"" 1>&2
  exit 1
fi


echo "modified src, finish!"
