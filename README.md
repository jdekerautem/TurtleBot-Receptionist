# TurtleBot-Receptionist
Details on the Speech-interface package respnsible for speech recognition and generation of Turtelbot

-- After the Pocketsphinx and sound_play packages installed, all the files need to be placed at /opt/ros/indigo/share/pocketsphinx/demo (If that's same with your system). 
    -> Don't forget to chmod +x ps_voice_nav.py

-- Whenever there is a change in the corpus, update the dictionary and the language model (.dic and .lm) files in the            ps_voice_nav_commands.launch file.

Once everything is set, open a Terminal and run the following:
    roslaunch pocketsphinx ps_voice_nav_commands.launch 
    
Open a new Terminal and run the following:
    rosrun sound_play soundplay_node.py

Open another Terminal and run the following:
    roslaunch pocketsphinx ps_turtlebot_voice_nav.launch
    
-- Now the robot should be ready to understand you and respond. 
