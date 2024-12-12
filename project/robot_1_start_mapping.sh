#!/bin/bash
# Script 1: Launching TurtleBot3 and related processes with tmux

# Start a tmux session
SESSION="turtlebot_session"
tmux new-session -d -s $SESSION

# Open the first tmux window and SSH into the TurtleBot3, providing the password and launching the robot bringup
echo "Creating Bringup window and SSHing into TurtleBot3..."
tmux rename-window -t $SESSION:0 "Bringup"
tmux send-keys -t $SESSION:0 "ssh ubuntu@192.168.1.151" C-m
sleep 3  # Allow time for SSH connection
echo "Sending password for TurtleBot3..."
tmux send-keys -t $SESSION:0 "turtlebot" C-\m
sleep 2
echo "Launching turtlebot3_bringup..."
tmux send-keys -t $SESSION:0 "roslaunch turtlebot3_bringup turtlebot3_robot.launch" C-m

# Open the second tmux window and launch the random mapper with the appropriate ROS workspace
echo "Creating Mapper window..."
tmux new-window -t $SESSION -n "Mapper"
tmux send-keys -t $SESSION:1 "source ~/ros_workspaces/ee106afinalproj/project/devel/setup.bash && roslaunch turtlebot_random_mapper robot1_mapper.launch" C-m
sleep 15



# Open the third tmux window and run the systematic mapping script
echo "Creating Mapping window..."
tmux new-window -t $SESSION -n "Mapping"
tmux send-keys -t $SESSION:2 "python /home/cc/ee106a/fa24/class/ee106a-aiv/ros_workspaces/ee106afinalproj/project/src/turtlebot_random_mapper/src/systematic_mapping.py" C-m
