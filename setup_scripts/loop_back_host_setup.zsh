#!/bin/zsh

cd ~/ros2-galactic-analysis

# check argument
if [ $# -ne 3 ]; then
  echo "usage: loop_back_host_setup.zsh <queue num> <msg size> <freq> " 1>&2
  echo "ex: loop_back_host_setup.zsh 10 1KB 30Hz" 1>&2
  exit 1
fi


# host publisher
# set backup queue size
if [[ "$1" =~ ^[0-9]+$ ]]; then
  sed -i "33c\    publisher_ = this->create_publisher<time_interface::msg::Timestamp>(\"topic1\", $1);" src/loop_back/src/host_publisher_member_function.cpp
else
  echo "use number for queue num. you set \"$1\"" 1>&2
  exit 1
fi

# set timer rate
if [ "$3" = "1Hz" ]; then
  sed -i "35c\      1000ms, std::bind(&MinimalPublisher::timer_callback, this));" src/loop_back/src/host_publisher_member_function.cpp
elif [ "$3" = "2Hz" ]; then
  sed -i "35c\      500ms, std::bind(&MinimalPublisher::timer_callback, this));" src/loop_back/src/host_publisher_member_function.cpp
elif [ "$3" = "10Hz" ]; then
  sed -i "35c\      100ms, std::bind(&MinimalPublisher::timer_callback, this));" src/loop_back/src/host_publisher_member_function.cpp
elif [ "$3" = "30Hz" ]; then
  sed -i "35c\      34ms, std::bind(&MinimalPublisher::timer_callback, this));" src/loop_back/src/host_publisher_member_function.cpp
elif [ "$3" = "100Hz" ]; then
  sed -i "35c\      10ms, std::bind(&MinimalPublisher::timer_callback, this));" src/loop_back/src/host_publisher_member_function.cpp
else
  echo "choose frequency from 1Hz, 2Hz, 10Hz, 30Hz, 100Hz"
  exit 1
fi


# set message size
if [ "$2" = "256B" ]; then
  sed -i "42c\    int array_size = 64;"  src/loop_back/src/host_publisher_member_function.cpp
elif [ "$2" = "1KB" ]; then
  sed -i "42c\    int array_size = 250;"  src/loop_back/src/host_publisher_member_function.cpp
elif [ "$2" = "10KB" ]; then
  sed -i "42c\    int array_size = 2500;"  src/loop_back/src/host_publisher_member_function.cpp
elif [ "$2" = "100KB" ]; then
  sed -i "42c\    int array_size = 25000;"  src/loop_back/src/host_publisher_member_function.cpp
elif [ "$2" = "500KB" ]; then
  sed -i "42c\    int array_size = 125000;"  src/loop_back/src/host_publisher_member_function.cpp
elif [ "$2" = "1MB" ]; then
  sed -i "42c\    int array_size = 250000;"  src/loop_back/src/host_publisher_member_function.cpp
elif [ "$2" = "2MB" ]; then
  sed -i "42c\    int array_size = 500000;"  src/loop_back/src/host_publisher_member_function.cpp
elif [ "$2" = "4MB" ]; then
  sed -i "42c\    int array_size = 1000000;"  src/loop_back/src/host_publisher_member_function.cpp
elif [ "$2" = "10MB" ]; then
  sed -i "42c\    int array_size = 2500000;"  src/loop_back/src/host_publisher_member_function.cpp
elif [ "$2" = "100MB" ]; then
  sed -i "42c\    int array_size = 25000000;"  src/loop_back/src/host_publisher_member_function.cpp
elif [ "$2" = "1GB" ]; then
  sed -i "49c\    int array_size = 250000000;"  src/loop_back/src/host_publisher_member_function.cpp
else
  echo "invalid message size, choose from 256B, 1KB, 10KB, 100KB, 500KB, 1MB, 2MB, 4MB"
  exit 1
fi

# host subscriber

rm -rf ./result/loop_back/result.txt
dir=`pwd`
mkdir -p ./result/loop_back
touch ./result/loop_back/result.txt
sed -i "54c\    std::ofstream outputfile(\"$dir/result/loop_back/result.txt\", std::ios::app);" src/loop_back/src/host_subscriber_member_function.cpp


echo "modified src, finish!"
