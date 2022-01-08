#!/bin/zsh

# check argument
if [ $# -ne 2 ]; then
  echo "usage: loop_back_remote_setup.zsh <queue num> <msg size>" 1>&2
  echo "ex: loop_back_remote_setup.zsh <10> <1KB>" 1>&2
  exit 1
fi

# set backup queue size
sed -i "28c\    publisher_ = this->create_publisher<time_interface::msg::Timestamp>(\"topic2\", $1);" src/loop_back/src/remote_member_function.cpp


# set message size
if [ "$2" = "100B" ]; then
  sed -i "37c\    int array_size = 25;"  src/loop_back/src/remote_member_function.cpp
elif [ "$2" = "1KB" ]; then
  sed -i "37c\    int array_size = 250;"  src/loop_back/src/remote_member_function.cpp
elif [ "$2" = "10KB" ]; then
  sed -i "37c\    int array_size = 2500;"  src/loop_back/src/remote_member_function.cpp
elif [ "$2" = "100KB" ]; then
  sed -i "37c\    int array_size = 25000;"  src/loop_back/src/remote_member_function.cpp
elif [ "$2" = "1MB" ]; then
  sed -i "37c\    int array_size = 250000;"  src/loop_back/src/remote_member_function.cpp
elif [ "$2" = "10MB" ]; then
  sed -i "37c\    int array_size = 2500000;"  src/loop_back/src/remote_member_function.cpp
elif [ "$2" = "100MB" ]; then
  sed -i "37c\    int array_size = 25000000;"  src/loop_back/src/remote_member_function.cpp
elif [ "$2" = "1GB" ]; then
  sed -i "37c\    int array_size = 250000000;"  src/loop_back/src/remote_member_function.cpp
else
  echo "choose message size from 100B ~ 1GB"
  exit 1
fi

echo "modified src, finish!"
