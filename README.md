# TurtleBot-Receptionist
Details on the Speech-interface package respnsible for speech recognition and generation of Turtelbot
    After the Pocketsphinx and sound_play packages installed, all the files need to be placed at:   
    /opt/ros/indigo/share/pocketsphinx/demo (If that's same with your system)
    
   
    Don't forget to:
    

    chmod +x talkative.py and notsotalkative.py

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
