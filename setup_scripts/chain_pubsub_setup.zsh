#!/bin/zsh

cd ~/ros2-galactic-analysis

# check argument
if [ $# -ne 4 ]; then
  echo "usage: chain_pubsub.zsh <queue num> <msg size> <freq> <inter node num>" 1>&2
  echo "ex: chain_pubsub.zsh 10 1KB 30Hz 3" 1>&2
  exit 1
fi


# set backup queue size
if [[ "$1" =~ ^[0-9]+$ ]]; then
  sed -i "22c\ #define QOS_HISTORY_SIZE $1" src/chain_pubsub/src/chain_pubsub.cpp
else
  echo "use number for queue num. you set \"$1\"" 1>&2
  exit 1
fi



# set timer rate
if [ "$3" = "1Hz" ]; then
  sed -i "39c\      1000ms, std::bind(&MinimalPublisher::timer_callback, this));" src/chain_pubsub/src/chain_pubsub.cpp
elif [ "$3" = "2Hz" ]; then
  sed -i "39c\      500ms, std::bind(&MinimalPublisher::timer_callback, this));" src/chain_pubsub/src/chain_pubsub.cpp
elif [ "$3" = "10Hz" ]; then
  sed -i "39c\      100ms, std::bind(&MinimalPublisher::timer_callback, this));" src/chain_pubsub/src/chain_pubsub.cpp
elif [ "$3" = "30Hz" ]; then
  sed -i "39c\      34ms, std::bind(&MinimalPublisher::timer_callback, this));" src/chain_pubsub/src/chain_pubsub.cpp
elif [ "$3" = "100Hz" ]; then
  sed -i "39c\      10ms, std::bind(&MinimalPublisher::timer_callback, this));" src/chain_pubsub/src/chain_pubsub.cpp
else
  echo "choose frequency from 1Hz, 2Hz, 10Hz, 30Hz, 100Hz"
  exit 1
fi


# set message size
if [ "$2" = "256B" ]; then
  sed -i "23c\ #define MESSAGE_ARRAY_SIZE 64"  src/chain_pubsub/src/chain_pubsub.cpp
elif [ "$2" = "1KB" ]; then
  sed -i "23c\ #define MESSAGE_ARRAY_SIZE 250;"  src/chain_pubsub/src/chain_pubsub.cpp
elif [ "$2" = "10KB" ]; then
  sed -i "23c\ #define MESSAGE_ARRAY_SIZE 2500;"  src/chain_pubsub/src/chain_pubsub.cpp
elif [ "$2" = "100KB" ]; then
  sed -i "23c\ #define MESSAGE_ARRAY_SIZE 25000;"  src/chain_pubsub/src/chain_pubsub.cpp
elif [ "$2" = "500KB" ]; then
  sed -i "23c\ #define MESSAGE_ARRAY_SIZE 125000;"  src/chain_pubsub/src/chain_pubsub.cpp
elif [ "$2" = "1MB" ]; then
  sed -i "23c\ #define MESSAGE_ARRAY_SIZE 250000;"  src/chain_pubsub/src/chain_pubsub.cpp
elif [ "$2" = "2MB" ]; then
  sed -i "23c\ #define MESSAGE_ARRAY_SIZE 500000;"  src/chain_pubsub/src/chain_pubsub.cpp
elif [ "$2" = "4MB" ]; then
  sed -i "23c\ #define MESSAGE_ARRAY_SIZE 1000000;"  src/chain_pubsub/src/chain_pubsub.cpp
elif [ "$2" = "10MB" ]; then
  sed -i "23c\ #define MESSAGE_ARRAY_SIZE 2500000;"  src/chain_pubsub/src/chain_pubsub.cpp
elif [ "$2" = "100MB" ]; then
  sed -i "23c\ #define MESSAGE_ARRAY_SIZE 25000000;"  src/chain_pubsub/src/chain_pubsub.cpp
elif [ "$2" = "1GB" ]; then
  sed -i "23c\ #define MESSAGE_ARRAY_SIZE 250000000;"  src/chain_pubsub/src/chain_pubsub.cpp
else
  echo "invalid message size, choose from 256B, 1KB, 10KB, 100KB, 500KB, 1MB, 2MB, 4MB"
  exit 1
fi

# set inter node num
if [[ "$4" =~ ^[0-9]+$ ]]; then
  sed -i "24c\ #define INTER_NODE_NUM $4"  src/chain_pubsub/src/chain_pubsub.cpp
else
  echo "use number for inter node num. you set \"$4\"" 1>&2
  exit 1
fi

echo "modified src, finish!"
