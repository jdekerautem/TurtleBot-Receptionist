#!/bin/sh

echo "Don't forget to source /opt/ros/indigo/setup.bash and your catkin workspace"
gnome-terminal --tab -e ./recognizer.sh --tab -e ./sound_play.sh --tab -e ./talkative.sh --tab -e ./webcamstream.sh --tab -e ./face_detection.sh --tab -e ./turtlebot_bringup.sh --tab -e ./steer_turtlebot.sh --tab -e ./rosserial_python.sh
#sleep 3
#gnome-terminal --tab -e ./sound_play.sh
#sleep 2
#gnome-terminal --tab -e ./talkative.sh
#sleep 2
#gnome-terminal --tab -e ./webcamstream.sh
#sleep 2
#gnome-terminal --tab -e ./face_detection.sh
#sleep 2
#gnome-terminal --tab -e ./turtlebot_bringup.sh
#sleep 2
#gnome-terminal --tab -e ./steer_turtlebot.sh
#sleep 2
#gnome-terminal --tab -e ./rosserial_python.sh
