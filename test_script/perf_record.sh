#!/bin/zsh

cd ~/ros2_foxy_analysis/test_script

PID=`ps aux | grep pubsub/listener | head -1 | awk '{print $2}'`

sudo perf record -F 999 -g -p $PID -- sleep 60

#sudo perf script | grep eprosima > ../result/ros2_subscriber_perf_record_result.txt 

#sudo rm perf.data
