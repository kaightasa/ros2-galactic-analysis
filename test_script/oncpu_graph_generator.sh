#!/bin/zsh

cd ~/ros2-galactic-analysis/test_script

PID=`ps aux | grep pubsub/talker | head -1 | awk '{print $2}'`


sudo perf record -F 999 -g -p $PID --call-graph dwarf -- sleep 120

sudo perf script > stack.txt

sudo cat stack.txt | ../../FlameGraph/stackcollapse-perf.pl > out.perf-folded
mkdir -p ../result/publisher/svg/1KB
sudo ../../FlameGraph/flamegraph.pl out.perf-folded > ../result/publisher/svg/30Hz/publisher_stack_trace_1KB_16sub.svg

rm -rf perf.data perf.data.old stack.txt out.perf-folded



