Santhosh Alladi
Eli Latocki
Nabil Khan

Project for CSci 5551 Box Packing using Baxter

Place the box_pack folder in your local catkin workspace.

System requirement:
Ubuntu 16.04
ROS Kinetic
Python 2.7
OpenCV 3.1.0
CvBridge 1.12
MoveIt

Steps for buiding program:
cd ~/catkin_ws
catkin_make

Steps for running program:

1. Open up a terminal and run
	roscore

2. Open up a second terminal and run
	./baxter.sh

3. Then run
	rosrun box_pack box_pack.py
