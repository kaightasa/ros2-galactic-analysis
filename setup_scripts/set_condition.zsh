#!/bin/zsh

cd ~/ros2-galactic-analysis

# check argument
if [ $# -ne 3 ]; then
  echo "usage: set_condition.zsh <queue num> <msg size> <freq> " 1>&2
  echo "ex: set_condition.zsh <10> <1KB> <30Hz>" 1>&2
  exit 1
fi


# set backup queue size
sed -i "34c\    publisher_ = this->create_publisher<std_msgs::msg::Int32MultiArray>(\"topic1\", $1);" src/pubsub/src/publisher_member_function.cpp


# set timer rate
if [ "$3" = "1Hz" ]; then
  sed -i "37c\      1000ms, std::bind(&MinimalPublisher::timer_callback, this));" src/pubsub/src/publisher_member_function.cpp
elif [ "$3" = "2Hz" ]; then
  sed -i "37c\      500ms, std::bind(&MinimalPublisher::timer_callback, this));" src/pubsub/src/publisher_member_function.cpp
elif [ "$3" = "10Hz" ]; then
  sed -i "37c\      100ms, std::bind(&MinimalPublisher::timer_callback, this));" src/pubsub/src/publisher_member_function.cpp
elif [ "$3" = "30Hz" ]; then
  sed -i "37c\      34ms, std::bind(&MinimalPublisher::timer_callback, this));" src/pubsub/src/publisher_member_function.cpp
elif [ "$3" = "100Hz" ]; then
  sed -i "37c\      10ms, std::bind(&MinimalPublisher::timer_callback, this));" src/pubsub/src/publisher_member_function.cpp
else
  echo "choose frequency from 1Hz, 2Hz, 10Hz, 30Hz, 50Hz"
  exit 1
fi


# set message size
if [ "$2" = "256B" ]; then
  sed -i "49c\    int array_size = 64;"  src/pubsub/src/publisher_member_function.cpp
  sed -i "40c\    int array_size = 64;"  src/pubsub/src/subscriber_member_function.cpp
elif [ "$2" = "1KB" ]; then
  sed -i "49c\    int array_size = 250;"  src/pubsub/src/publisher_member_function.cpp
  sed -i "40c\    int array_size = 250;"  src/pubsub/src/subscriber_member_function.cpp
elif [ "$2" = "10KB" ]; then
  sed -i "49c\    int array_size = 2500;"  src/pubsub/src/publisher_member_function.cpp
  sed -i "40c\    int array_size = 2500;"  src/pubsub/src/subscriber_member_function.cpp
elif [ "$2" = "100KB" ]; then
  sed -i "49c\    int array_size = 25000;"  src/pubsub/src/publisher_member_function.cpp
  sed -i "40c\    int array_size = 25000;"  src/pubsub/src/subscriber_member_function.cpp
elif [ "$2" = "500KB" ]; then
  sed -i "49c\    int array_size = 125000;"  src/pubsub/src/publisher_member_function.cpp
  sed -i "40c\    int array_size = 125000;"  src/pubsub/src/subscriber_member_function.cpp
elif [ "$2" = "1MB" ]; then
  sed -i "49c\    int array_size = 250000;"  src/pubsub/src/publisher_member_function.cpp
  sed -i "40c\    int array_size = 250000;"  src/pubsub/src/subscriber_member_function.cpp
elif [ "$2" = "2MB" ]; then
  sed -i "49c\    int array_size = 500000;"  src/pubsub/src/publisher_member_function.cpp
  sed -i "40c\    int array_size = 500000;"  src/pubsub/src/subscriber_member_function.cpp
elif [ "$2" = "4MB" ]; then
  sed -i "49c\    int array_size = 1000000;"  src/pubsub/src/publisher_member_function.cpp
  sed -i "40c\    int array_size = 1000000;"  src/pubsub/src/subscriber_member_function.cpp
elif [ "$2" = "10MB" ]; then
  sed -i "49c\    int array_size = 2500000;"  src/pubsub/src/publisher_member_function.cpp
  sed -i "40c\    int array_size = 2500000;"  src/pubsub/src/subscriber_member_function.cpp
elif [ "$2" = "100MB" ]; then
  sed -i "49c\    int array_size = 25000000;"  src/pubsub/src/publisher_member_function.cpp
  sed -i "40c\    int array_size = 25000000;"  src/pubsub/src/subscriber_member_function.cpp
elif [ "$2" = "1GB" ]; then
  sed -i "49c\    int array_size = 250000000;"  src/pubsub/src/publisher_member_function.cpp
  sed -i "40c\    int array_size = 250000000;"  src/pubsub/src/subscriber_member_function.cpp
else
  echo "invalid message size, choose from 256B, 1KB, 10KB, 100KB, 500KB, 1MB, 2MB, 4MB"
  exit 1
fi

echo "modified src, finish!"
