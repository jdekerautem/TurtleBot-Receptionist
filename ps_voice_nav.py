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
                                    'bye': ['bye', 'cheers', 'goodbye', 'see you', 'bye'],
                                    'cafe' : ['cafe', 'campus'],
                                    'hello': ['hi', 'hey', 'hello'],
                                    'help' : ['help me', 'can help', 'help'],
                                    'name' : ['your name', 'name'],                             
                                    'what' : ['what is', 'what is your', 'what'],
                                    'wash' : ['washroom', 'toilet'],
                                    'library' : ['library'],
                                    'labs' : ['labs'],
                                    'talk': ['talk to me?', 'really talk?', 'you talk', 'you really talk?', 'talk'],                      
                                    'how' : ['can you', 'can', 'how can', 'how can you'],
                                    'cute' : ['cute', 'so cute'],
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
            self.soundhandle.say("Greetings! from your very own Charlie. How may I assist you?.", self.voice)
            rospy.sleep(3)
            
        elif command == 'bye':
        	self.soundhandle.say("Goodbye! See you soon.", self.voice)
        	rospy.sleep(2)
        
        if command == 'help':
        	self.soundhandle.say("I am a receptionist robot. I can answer your questions about the faculty. or guide you through the faculty building.", self.voice)
        	
                    	
        if command == 'talk':
        	self.soundhandle.say("Oh! I am talking.. Have you seen my brother Wall E?.", self.voice)
        	        	
		
        
        if command == 'cute':
        	self.soundhandle.say("Oh my God! I am blushing. You are very kind", self.voice)
            
        if command == 'name':
        	self.soundhandle.say("They named me charlie. As if I wanted it. ", self.voice)
       	
       	if command == 'amazing':
       		self.soundhandle.say("Not as much as you are. But thank you so much.", self.voice)           
        
        if command == 'cafe':
        	self.soundhandle.say("There is one S U shop right opposite to the library. you can eat or have coffee there", self.voice)      
       
        if command == 'wash':
        	self.soundhandle.say("The Gents toilet can be found on the second floor. and a ladies toilet on the third floor.", self.voice)
    
        if command == 'library':
        	self.soundhandle.say("As you will exit out of the Smeaton's building. Take a left. The building in front of you is the library. ", self.voice)
        
        if command == 'labs':
        	self.soundhandle.say("The labs are on third floor of the Smeaton's building. Infact, we might be standing in one of those. You bet, if i am wrong. ", self.voice)
    
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
