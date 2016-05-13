# TurtleBot-Receptionist
Details on the Speech-interface package respnsible for speech recognition and generation of Turtelbot. 

After the Pocketsphinx and sound_play packages installed, all the files need to be placed at: 

<<<<<<< HEAD
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

```

Details on the Speech-interface package responsible for speech recognition and generation of Turtelbot. 

After the Pocketsphinx and sound_play packages installed, all the files need to be placed at: 
=======
      
>>>>>>> Speech-interface
    /opt/ros/indigo/share/pocketsphinx/demo (If that's same with your system)
    
-- Don't forget to:
   

    chmod +x talkative.py
    chmod +x notsotalkative.py

-- Whenever there is a change in the corpus, update the dictionary and the language model (.dic and .lm) files in the following:   

    talkative_3.launch
    notsotalkative_3.launch 

Once everything is set, open a Terminal and run the following:

    roslaunch pocketsphinx recognizer_1.launch 
    
Open a new Terminal and run the following:

    rosrun sound_play soundplay_node.py

Open another Terminal and run the following based on what you want to run:

    roslaunch pocketsphinx talkative_3.launch
    roslaunch pocketsphinx notsotalkative_3.launch  
    
-- Now the robot should be ready to understand you and respond. 
