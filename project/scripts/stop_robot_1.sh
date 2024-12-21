#!/bin/bash
# Script 2: Run camera disconnect script and cancel the bringup process

echo "Launching camera error!!!"
python /home/cc/ee106a/fa24/class/ee106a-aiv/ros_workspaces/ee106afinalproj/project/src/detect/src/camera_disconnect.py

echo "Camera broke down!!!"

sleep 3

# Cancel the bringup process
tmux kill-session -t turtlebot_session:0

# Command to kill everything (detach and kill tmux session)
# To stop all processes: tmux kill-session -t $SESSION