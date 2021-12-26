## perf analysis initialization
1. `$ perf_pubsub`
to run this command, you need to set function below to .bashrc or .zshenv

## caret analysis initialization
1. `$ caret_pubsub`  
to run this command, you need to set function below to .bashrc or .zshenv

```
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

function caret_pubsub() {
  source ~/ros2_galactic/install/setup.zsh;
  source ~/ros2_caret_ws/install/local_setup.zsh;
  source ~/ros2-galactic-analysis/install/local_setup.zsh;
  export LD_PRELOAD=$(readlink -f ~/ros2_caret_ws/install/caret_trace/lib/libcaret.so);
  export CARET_IGNORE_NODES="/rviz*";
  export CARET_IGNORE_TOPICS="/clock:/parameter_events";
}

export -f caret_pubsub # only need this for .bashrc
alias caret_pubsub=caret_pubsub
```
# launch

1. `$ ros2 launch ./src/lauch/pubsub_launch.py`
