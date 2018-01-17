#!/bin/sh
cd ~/catkin_ws
roscore &
sleep 3
xterm -e "bash -c \"rostopic echo /rosout > tenderLog.log; exec bash\"" &
sleep 1
catkin_make
sleep 1
xterm -e "bash -c \"roslaunch tenderbot TenderBot.launch; exec bash\"" &
sleep 1
python ~/catkin_ws/src/ITROB-LAB1/tenderbot_vision/opencvlibrary/visiongui.py ros
cd -
./killAll.sh
