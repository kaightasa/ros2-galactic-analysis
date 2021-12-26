#!/bin/zsh

PID=`ps aux | grep pubsub/talker | head -1 | awk '{print $2}'`

sudo perf record -F 999 -p $PID --call-graph dwarf -- sleep 30
sudo perf report -g -G > perf_report.txt
#sudo perf script | grep eprosima > ../result/ros2_subscriber_perf_record_result.txt 
#sudo perf script > ../result/ros2_subscriber_perf_record_result-all.txt 
#sudo perf report -G > ../result/ros2_subscriber_perf_report_result-all.txt

#sudo rm perf.data
