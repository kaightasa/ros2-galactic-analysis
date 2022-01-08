#!/bin/zsh

cd ~/ros2-galactic-analysis
rm -rf ./result/remote_pubsub/result.txt
dir=`pwd`
mkdir -p ./result/remote_pubsub
touch ./result/remote_pubsub/result.txt
sed -i "54c\    std::ofstream outputfile(\"$dir/result/remote_pubsub/result.txt\", std::ios::app);" src/remote_pubsub/src/subscriber_member_function.cpp

#TODO rename outputfile (msgsize, rmw name)
