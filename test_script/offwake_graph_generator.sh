#!/bin/zsh

cd ~/ros2_foxy_analysis/test_script

PID=`ps aux | grep pubsub/listener | head -1 | awk '{print $2}'`

sudo ~/bcc/tools/offwaketime.py -f 60 -p $PID > out.stacks

~/FlameGraph/flamegraph.pl --color=chain --title="Off-Wake Time Flame Graph" --countname=us < out.stacks > offwaketime.svg
