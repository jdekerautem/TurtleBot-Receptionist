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
In case of turtlebot bringup throwing an error of name of base uninitialized, run the following commands:
```sh
. /opt/ros/indigo/setup.bash 
rosrun kobuki_ftdi create_udev_rules
```

--> To remote-control turtlebot with an Android smartphone, download the app ROS Teleop (Indigo) from Google Play Store.
After setting the URI of master node to the IP address of the computer:
```sh
export ROS_MASTER_URI=http://write.your.IP.address:11311
```
Then launch the node remocon:
```sh
roslaunch remocon_turtlebot remocon.launch
```
If you want to create your own wifi network on your ubuntu computer, follow [this link](http://askubuntu.com/questions/490950/create-wifi-hotspot-on-ubuntu)


--> To run face_detection:
Edit CMakeLists.txt: modify SET(OpenCV_INCLUDE_DIRS ) accordingly to your installation

Before running any file, and for any new terminal you open, source your catkin workspace:

    source your_catkin_workspace/devel/setup.bash

How to run FACE_DETECTION -- Make sure you are in catkin workspace directory:

    roslaunch webcamstream uvc_cam.launch
    rosrun ros_opencv_converter face_detection
