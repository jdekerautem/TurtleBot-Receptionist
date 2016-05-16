# TurtleBot-Receptionist
This project consists in implementing a receptionist based on the turtlebot by Kobuki. This receptionist can also be used as a charity fundraiser. 

The face tracking ability of this robot is based on Opencv 3.0 libraries.

ROS Packages (indigo) needed in order to have the nodes run:
  - openni2_launch (for using the Asus Xtion kinect-like sensor. However not used in this project)
  - pocketsphinx
  - sound_play
  - rosserial_server
  - rosserial_arduino
  - rosserial_python
  - uvc_camera
  - turtlebot-bringup
  - turtlebot-teleop (for remote-controlling the turtlebot)

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

-----> To run face_detection:

Edit CMakeLists.txt: modify SET(OpenCV_INCLUDE_DIRS ) accordingly to your installation

Before running any file, and for any new terminal you open, source your catkin workspace:

    source your_catkin_workspace/devel/setup.bash

How to run FACE_DETECTION -- Make sure you are in catkin workspace directory, and that your webcam is plugged in.
In case you have more than one camera plugged into the computer, you would need to choose which one to use by modifying 
the parameter "value" at line 7 of webcamstream/launch/uvcCamLaunch.launch (default value: /dev/video0). Then run:

    roslaunch webcamstream uvc_cam.launch
    rosrun ros_opencv_converter face_detection


-----> Details on the Speech-interface package responsible for speech recognition and generation of Turtelbot. 

After the Pocketsphinx and sound_play packages installed, all the files need to be placed at:

    /opt/ros/indigo/share/pocketsphinx/demo (If that's same with your system)
    
 Don't forget to:

    chmod +x talkative.py
    chmod +x notsotalkative.py

 Whenever there is a change in the corpus, update the dictionary and the language model (.dic and .lm) files in the following:
 ```sh
    talkative_3.launch
    notsotalkative_3.launch 
```
 Once everything is set, open a Terminal and run the following:

    roslaunch pocketsphinx recognizer_1.launch 
    
 Open a new Terminal and run the following:

    rosrun sound_play soundplay_node.py

 Open another Terminal and run the following based on what you want to run:

    roslaunch pocketsphinx talkative_3.launch
    roslaunch pocketsphinx notsotalkative_3.launch  
    
 Now the robot should be ready to understand you and respond. 


-----> To remote-control turtlebot via ssh, you need to have openssh-server and openssh-client installed on your laptop.
On the local computer, set the URI of master node to the IP address of the local computer, on which all the nodes will be running:
```sh
export ROS_MASTER_URI=http://write.your.IP.address:11311
```
Then, on the remote laptop, launch the node turtlebot_teleop to steer the platform:
```sh
roslaunch turtlebot-teleop keyboard_teleop.launch
```
In order to play the wav files from the remote laptop, first put the folder containing the wav files in the /opt/ros/indigo/share/pockestphinx folder on the local computer. Then, on the remote laptop:
```sh
rosrun sound_play sound_play_node.py
rosrun sound_play play.py /opt/ros/indigo/share/pocketsphinx/wav_files/normal_hello.wav
```
for example.

If you want to create your own wifi network on your ubuntu computer, follow [this link](http://askubuntu.com/questions/490950/create-wifi-hotspot-on-ubuntu)
