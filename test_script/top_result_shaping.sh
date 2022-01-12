#!/bin/zsh

if [ $# -ne 1 ]; then
  echo "usage: top_result_shaping.sh  <application name>" 1>&2
  echo "ex: top_result_shaping.sh pubsub" 1>&2
  exit 1
fi

rm ~/ros2-galactic-analysis/graph_scripts/top_result.txt
touch ~/ros2-galactic-analysis/graph_scripts/top_result.txt

echo "RES,CPU,subnum,msgsz" > ~/ros2-galactic-analysis/graph_scripts/top_result.txt



dir=/home/asana/ros2-galactic-analysis/result/$1/top
for filepath in $dir/*.txt; do
  tname=${filepath##*/}
  output=${tname%.*}
  subnum=${output##*_}
  t1msgsz=${output#*_}
  t2msgsz=${t1msgsz#*_}
  msgsz=${t2msgsz%%_*}

  #RES
  RES=`cat $filepath | awk '{m+=$7} END { print m/NR;}'`
  CPU=`cat $filepath | awk '{m+=$10} END { print m/NR;}'`
  echo "$RES,$CPU,$subnum,$msgsz" >> ~/ros2-galactic-analysis/graph_scripts/top_result.txt

done
