#!/bin/zsh

cd ~/ros2_foxy_analysis/test_script

PID=`ps aux | grep pubsub/listener | head -1 | awk '{print $2}'`

sudo perf record -F 999 -g -p $PID --call-graph dwarf -- sleep 30

sudo perf script | awk 'BEGIN{RS="";FS="\n";ORS="\n\n";OFS="\n"}/eprosima/{print $0 }' > extracted_stack.txt

sudo cat extracted_stack.txt | ../../FlameGraph/stackcollapse-perf.pl > out.perf-folded
sudo ../../FlameGraph/flamegraph.pl out.perf-folded > subscriber_eprosima_stack_trace.svg

rm -rf perf.data extracted_stack.txt out.perf-folded



