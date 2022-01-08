## build instruction
### pubsub
```
$ source /opt/ros/galactic/setup.zsh
$ rosdep install -i --from-path src --rosdistro galactic -y
$ colcon build --packages-select pubsub
```
### remote_pubsub
```
$ source /opt/ros/galactic/setup.zsh
$ rosdep install -i --from-path src --rosdistro galactic -y
$ colcon build --packages-select time_interface
$ source ~/ros2-galactic-analysis/install/local_setup.zsh
$ colcon build --packages-select remote_pubsub
```
## perf analysis initialization
1. `$ perf_pubsub`  
to run this command, you need to set function below to .bashrc or .zshenv

```
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

function perf_pubsub() {
  source ~/ros2_galactic/install/setup.zsh; # you need to build ros2 from source
  source ~/ros2-galactic-analysis/install/local_setup.zsh;
}

export -f perf_pubsub # only need this for .bashrc
alias perf_pubsub=caret_pubsub
```

## remote pubsub initialization
1. synchronize time  
ex. use chrony  
    1. install chrony ` $ sudo apt install chrony`
    2. modify server config `$ sudo vim /etc/chrony/chrony.conf`
    ```
    # comment out 'pool' and 'server' line
    # add following lines
    makestep 0.1 10
    server ntp.nict.jp iburst prefer
    ```
    3. modify client config
    ```
    # comment out 'pool' and 'server' line
    # add following lines
    makestep 0.1 10
    server <server ip address> iburst prefer
    ```
    4. confirm setting  
    `$ chronyc sources`  
    `$ sudo chronyc -a clients # on server`

2. `remote_pubsub()`  
to run this command, add function below to .bashrc or .zshenv
```
function remote_pubsub() {
  source /opt/ros/galactic/setup.zsh;
  source ~/ros2-galactic-analysis/install/local_setup.zsh;
  export ROS_DOMAIN_ID=1;
}

alias remote_pubsub=remote_pubsub
```
3. run setup scripts.
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

## caret analysis memo
### step1 get lttng data
#### lttng session terminal
```
source /opt/ros/galactic/setup.zsh
export ROS_TRACE_DIR=~/ros2-galactic-analysis/evaluate
rm ~/ros2-galactic-analysis/evaluate/pubsub1
ros2 trace -s pubsub1 -k -u "ros2*"
```
#### application terminal
```
caret_pubsub
ros2 launch src/launch/pubsub_sub1_launch.py
```
### step2 analyze data on jupyter-lab
```
babeltrace ~/ros2-galactic-analysis/evaluate/pubsub1/ | cut -d' ' -f 4 | sort -u  # check if trace is fine
cd ~/ros2-galactic-analysis/evaluate
source ~/ros2_caret_ws/install/setup.zsh
jupyter-lab
```
After this, use `architecture ipynb` and `analysis.ipynb`

### extra step: change variables
1. use `set_condition.zsh` to change application variables
2. source ros&caret `local_setup.zsh` and build
```
source ~/ros2_caret_ws/install/setup.zsh
colcon build --pakages-select pubsub
```
3. when analyze data, don't forget to restart jupyter kernel
