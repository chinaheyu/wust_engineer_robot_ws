#!/usr/bin/bash
echo "Removing remote file ..."
sshpass -p dji ssh dji@192.168.1.124 "rm -rf rm_auto_ws/build/ rm_auto_ws/devel/ rm_auto_ws/src/"
echo "Uploding file to remote ..."
sshpass -p dji scp -r src/ dji@192.168.1.124:/home/dji/rm_auto_ws
echo "Compiling the code ..."
sshpass -p dji ssh dji@192.168.1.124 "source /opt/ros/kinetic/setup.bash && cd rm_auto_ws/ && catkin_make"
echo "Finished."