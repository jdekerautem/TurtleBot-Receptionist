# TurtleBot-Receptionist
Implementation of a receptionist based on the turtlebot platform

This project runs on OpenCV 3.0 libraries

ROS Packages (indigo) needed in order to have the nodes run:
  - openni2_launch
  - pocketsphinx
  - sound_play
  - rosserial_server
  - rosserial_arduino
  - rosserial_python
  - uvc_camera
  - turtlebot-bringup

In order to launch all the nodes from a unique command, put all the files in the "shell script" folder
in the root of your catkin workspace. You still need to source ros and your workspace:
```sh
source /opt/ros/indido/setup.bash
source catkin_workspace/devel/setup.bash
cd catkin_workspace
chmod +x *.sh
./initialisation_script_talkative.sh OR ./initialisation_script_NOTsotalkative.sh
```
