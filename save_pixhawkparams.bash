#!/bin/bash 
current_time=$(date "+%Y_%m_%d_%H_%M_%S")
file_name=pixhawk_params_$current_time.yaml
echo "Writing Pixhawk Params to $file_name"
rosrun dynamic_reconfigure dynparam dump /onboard_node/pixhawk_tuning $file_name
