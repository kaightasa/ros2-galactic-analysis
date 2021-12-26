#!/bin/zsh

cd ~/ros2-galactic-analysis/test_script

PID=`ps aux | grep pubsub/talker | head -1 | awk '{print $2}'`


sudo perf record -F 999 -g -p $PID --call-graph dwarf -- sleep 30

sudo perf script > stack.txt 

sudo cat stack.txt | ../../FlameGraph/stackcollapse-perf.pl > out.perf-folded
sudo ../../FlameGraph/flamegraph.pl out.perf-folded > ../result/publisher/svg/1MB/publisher_stack_trace_1MB.svg

rm -rf perf.data stack.txt out.perf-folded



