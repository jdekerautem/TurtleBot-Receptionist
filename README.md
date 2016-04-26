# TurtleBot-Receptionist
Implementation of a receptionist based on the turtlebot platform

Edit CMakeLists.txt: modify SET(OpenCV_INCLUDE_DIRS ) accordingly to your installation

Before running any file, and for any new terminal you open, source your catkin workspace:

    source your_catkin_workspace/devel/setup.bash

How to run the different programs:
FACE_DETECTION -- Make sure you are in catkin workspace directory

    roslaunch webcamstream uvc_cam.launch
    rosrun ros_opencv_converter face_detection
