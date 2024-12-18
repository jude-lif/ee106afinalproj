#!/bin/bash

# Define SSH credentials and ROS workspace paths
SSH_USER="apple"
SSH_HOST="apple"
SSH_PASS="apple2022"

# Start a new tmux session
tmux new-session -d -s turtlebot3_setup

# SSH into the remote machine
tmux send-keys -t turtlebot3_setup "ssh $SSH_USER@$SSH_HOST" C-m
sleep 3
tmux send-keys -t turtlebot3_setup "apple2022" C-m
sleep 2
echo "Launching Turtlebot Bring Up"
tmux send-keys -t turtlebot3_setup "roslaunch turtlebot3_bringup turtlebot3_robot.launch" C-m
sleep 1

# Open another pane for the Realsense2 Camera launch
tmux split-window -h
tmux send-keys -t 0 "ssh $SSH_USER@$SSH_HOST" C-m
sleep 3
echo "Launching Camera Bring Up"
tmux send-keys -t 0 "roslaunch realsense2_camera rs_camera.launch mode:=Manual color_width:=424 color_height:=240 depth_width:=424 depth_height:=240 align_depth:=true depth_fps:=6 color_fps:=6" C-m
sleep 1

# Open another pane for Object Detection
tmux split-window -v
echo "Launching Object Detector Script"
tmux send-keys -t 1 "source ~/ros_workspaces/ee106afinalproj/project/devel/setup.bash" C-m
tmux send-keys -t 1 "cd ~/ros_workspaces/ee106afinalproj/project/" C-m
tmux send-keys -t 1 "rosrun perception object_detector.py" C-m

sleep 1

# Open another pane for map server
tmux split-window -v
echo Launching Map Server""
tmux send-keys -t 2 "rosrun map_server map_server /home/cc/ee106a/fa24/class/ee106a-aiv/ros_workspaces/ee106afinalproj/project/src/map.yaml" C-m
sleep 1

# Open another pane for navigation
tmux split-window -h
echo "Launching Navigation Node"
tmux send-keys -t 3 "roslaunch turtlebot3_navigation turtlebot3_navigation.launch map_file:=/home/cc/ee106a/fa24/class/ee106a-aiv/ros_workspaces/ee106afinalproj/project/src/map.yaml" C-m
sleep 20

# Open another pane for running the navigation script
tmux split-window -v
echo "Looking for Failed Turtlebot!"
tmux send-keys -t 4 "python /home/cc/ee106a/fa24/class/ee106a-aiv/ros_workspaces/ee106afinalproj/project/src/turtlebot_random_mapper/src/load_map_and_naavigate.py" C-m

sleep 20
# Attach the session for the user to interact: kill-session -t turtlebot3_setup:0
tmux attach-session -t turtlebot3_setup

