#!/usr/bin/env python

"""
  voice_nav.py allows controlling a mobile base using simple speech commands.
  Based on the voice_cmd_vel.py script by Michael Ferguson in the pocketsphinx ROS package.
"""

import roslib; #roslib.load_manifest('pi_speech_tutorial')
import rospy

from geometry_msgs.msg import Twist
from std_msgs.msg import String
from math import copysign

from sound_play.libsoundplay import SoundClient

class voice_cmd_vel:
    def __init__(self):
        rospy.on_shutdown(self.cleanup)
        self.max_speed = rospy.get_param("~max_speed", 0.4)
        self.max_angular_speed = rospy.get_param("~max_angular_speed", 1.5)
        self.speed = rospy.get_param("~start_speed", 0.1)
        self.angular_speed = rospy.get_param("~start_angular_speed", 0.5)
        self.linear_increment = rospy.get_param("~linear_increment", 0.05)
        self.angular_increment = rospy.get_param("~angular_increment", 0.4)
        self.rate = rospy.get_param("~rate", 5)
        r = rospy.Rate(self.rate)
        self.paused = False
        
        self.voice = rospy.get_param("~voice", "voice_cmu_us_bdl_arctic_clunits")
        self.wavepath = rospy.get_param("~wavepath", "")
        
        # Create the sound client object
        self.soundhandle = SoundClient()
        
        rospy.sleep(1)
        self.soundhandle.stopAll()
        
        
        
        # Initialize the Twist message we will publish.
        self.msg = Twist()

        # Publish the Twist message to the cmd_vel topic
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        
        # Subscribe to the /recognizer/output topic to receive voice commands.
        rospy.Subscriber('/recognizer/output', String, self.speechCb)
        
        # A mapping from keywords to commands.
        self.keywords_to_command = {'stop': ['stop', 'halt', 'abort', 'kill', 'panic', 'off', 'freeze', 'shut down', 'turn off', 'help', 'help me'],
                                    'hello': ['hi', 'hey', 'hello'],
                                    'help' : ['help me', 'can help', 'help'],
                                    'name' : ['your name', 'name'],                             
                                    'how' : ['how can', 'how can you', 'how you doing'],
                                    'what' : ['what is', 'what is your', 'what'],
                                    'talk': ['talk to me?', 'really talk?', 'you talk', 'you really talk?', 'talk'],                      
                                    'can' : ['can you', 'can'],
                                    'cute' : ['cute', 'so cute'],
                                    'creepy' : ['creepy', 'wierd'],
                                    'amazing' : ['amazing', 'wonderful'],                                    
                                    'pause': ['pause speech'],
                                    'continue': ['continue speech']}
        
        rospy.loginfo("Ready to receive voice commands")
        
        # We have to keep publishing the cmd_vel message if we want the robot to keep moving.
        while not rospy.is_shutdown():
            self.cmd_vel_pub.publish(self.msg)
            r.sleep()                       
            
    	
            
    def get_command(self, data):
        for (command, keywords) in self.keywords_to_command.iteritems():
            for word in keywords:
                if data.find(word) > -1:
                    return command
        
    def speechCb(self, msg):        
        command = self.get_command(msg.data)
        
        rospy.loginfo("Command: " + str(command))
        
        if command == 'pause':
            self.paused = True
        elif command == 'continue':
            self.paused = False
            
        if self.paused:
            return       
        
        if command == 'hello':    
            self.soundhandle.say("Greetings from your very own Charlie", self.voice)
            
        if command == 'can' and 'help':
        	self.soundhandle.say("Sure, how may I help", self.voice)
        
        if command == 'how':    
            self.soundhandle.say("Well thank you, this is my first day. How about you", self.voice)
                    	
        if command == 'talk':
        	self.soundhandle.say("Oh! I am talking. Have you seen my brother Wall E", self.voice)
        	        	
 		if command == 'creepy':
 			self.soundhandle.say("Oh! I am very hurt. Somebody call my mom", self.voice)
        
        if command == 'cute':
        	self.soundhandle.say("Oh my God! I am blushing. You are very kind", self.voice)
            
        if command == 'name':
        	self.soundhandle.say("My name is charlie. And I love it", self.voice)
       	
       	if command == 'amazing':
       		self.soundhandle.say("Not as much as you are. But thank you.", self.voice)           
                
       
    def cleanup(self):
        # When shutting down be sure to stop the robot!  Publish a Twist message consisting of all zeros.
        twist = Twist()
        self.cmd_vel_pub.publish(twist)
		
		
if __name__=="__main__":
    rospy.init_node('voice_nav')
    
    try:
        voice_cmd_vel()
    except:
        pass

