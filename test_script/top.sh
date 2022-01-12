#!/bin/zsh

cd ~/ros2-galactic-analysis

# check argument
if [ $# -ne 4 ]; then
  echo "usage: top.sh  <app name> <msg size> <freq> <inter/subscriber node num>" 1>&2
  echo "ex: top.sh pubsub 1KB 30Hz 3" 1>&2
  exit 1
fi

mkdir -p result/$1/top/

#top -d 1 | grep --line-buffered chain_pubsub | tee result/$1/top/top_result_${2}_${3}_$4.txt
top -d 1 | grep --line-buffered talker | tee result/$1/top/top_result_${2}_${3}_$4.txt
