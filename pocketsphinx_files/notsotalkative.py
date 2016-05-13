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
        
        self.rate = rospy.get_param("~rate", 5)
        r = rospy.Rate(self.rate)
        self.paused = False
        
        self.voice = rospy.get_param("~voice", "voice_cmu_us_bdl_arctic_clunits")
        self.wavepath = rospy.get_param("~wavepath", "")
        
        # Create the sound client object
        self.soundhandle = SoundClient()
        
        rospy.sleep(1)
        self.soundhandle.stopAll()
        
        
        # Subscribe to the /recognizer/output topic to receive voice commands.
        rospy.Subscriber('/recognizer/output', String, self.speechCb)
        
        # A mapping from keywords to commands.
        self.keywords_to_command = {'stop': ['stop', 'halt', 'abort', 'kill', 'panic', 'off', 'freeze', 'shut down', 'turn off', 'help', 'help me'],
                                    'bye': ['bye', 'cheers', 'goodbye', 'see you', 'bye'],
                                    'cafe' : ['cafe', 'campus', 'tea', 'coffee', 'eat'],
				    'hello': ['hi', 'hey', 'hello'],
                                    'help' : ['help me', 'can help', 'help'],
                                    'name' : ['your name', 'name'],                             
                                    'wash' : ['washroom', 'toilet'],
                                    'library' : ['library', 'book', 'borrow'],
                                    'labs' : ['labs'],
                                    'talk': ['talk to me?', 'really talk?', 'you talk', 'you really talk?', 'talk'],                      
                                    'amazing' : ['amazing', 'wonderful'], 
				    'psychology' : ['psychology'],
				    'teaching' : ['teaching', 'music'],
				    'engineering' : ['engineering'],
				    'biology' : ['biology', 'english', 'chemistry'],
				    'maths' : ['computing', 'mathematics'],
				    'geo' : ['geology', 'geography'],
				    'marine' : ['marine'],
				    'art' : ['art'],
    				    'roland' : ['reception', 'architecture'],
				    'business' : ['business'],
				    'staff' : ['staff'],
				    'sports' : ['sports'],
  				    'robots' : ['robotics', 'robots'],
				    'visit' : ['visit', 'to do'],
				    'supermarket' : ['shop', 'supermarket'],
			     	    'cashpoint' : ['cash points', 'ATM', 'cash machines'],
				    'day' : ['day', 'today'],
				    'weather' : ['weather'],                                 
                                    'pause': ['pause speech'],
                                    'continue': ['continue speech']}
        
        rospy.loginfo("Ready to receive voice commands")
        
        # We have to keep publishing the cmd_vel message if we want the robot to keep moving.
        while not rospy.is_shutdown():
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
		self.soundhandle.say("Greetings!.", self.voice)
                
        if command == 'help':
		self.soundhandle.say("Ask me questions", self.voice)
                    	
        if command == 'talk':
        	self.soundhandle.say("yes, I can", self.voice)
        	        	
 	if command == 'bye':
 		self.soundhandle.say("Bye Bye", self.voice)
        
        if command == 'weather':
		self.soundhandle.say("I Don't know.", self.voice)
        
	if command == 'supermarket':
		self.soundhandle.say("The nearest supermarket is the TESCO!. ", self.voice)
	
	if command == 'day':
		self.soundhandle.say("It's tuesday!.", self.voice)

	if command == 'psychology':
		self.soundhandle.say("It's in link building!", self.voice)

	if command == 'teaching':
		self.soundhandle.say("the rolle building!.", self.voice)

	if command == 'engineering':
		self.soundhandle.say("That's right here!.", self.voice)

	if command == 'biology':
		self.soundhandle.say("It's is in the Davy building!.!", self.voice)
	
	if command == 'maths':
		self.soundhandle.say("In the babbage building!.!", self.voice)

	if command == 'geo':
		self.soundhandle.say("It's in the Fitzroy building!.!", self.voice)

	if command == 'marine':
		self.soundhandle.say("In the reynolds And the marine building.! ", self.voice)
	
	if command == 'art':
		self.soundhandle.say(" in the scott building!.!", self.voice)
	
	if command == 'roland':
		self.soundhandle.say(" in the roland levinsky building!.!", self.voice)
	
	if command == 'business':
		self.soundhandle.say("should be cookworthy building!", self.voice)

	if command == 'staff':
		self.soundhandle.say("In the Portland Square building!", self.voice)

	if command == 'sports':
		self.soundhandle.say("It's the Nancy Astor building. ", self.voice)

	if command == 'robots':
		self.soundhandle.say("in Smeaton's building or in Portland Square. !", self.voice)	

	if command == 'cashpoint':
		self.soundhandle.say("There are some on the eastern exit of this building.!!", self.voice)

	if command == 'visit':
		self.soundhandle.say("Well, you can walk along the seashore. May be.!", self.voice)

            
        if command == 'name':
        	self.soundhandle.say("charlie.", self.voice)
       	
       	if command == 'amazing':
       		self.soundhandle.say("thank you so much.", self.voice)           
        
        if command == 'cafe':
        	self.soundhandle.say(" at the S U shop.", self.voice)      
       
        if command == 'wash':
        	self.soundhandle.say("the second floor and the third floor.", self.voice)
    
        if command == 'library':
        	self.soundhandle.say("It's next to the Smeaton's building.", self.voice)
        
        if command == 'labs':
        	self.soundhandle.say(" on the third floor.", self.voice)

	

	
    
    def cleanup(self):
        # When shutting down be sure to stop the robot!  Publish a Twist message consisting of all zeros.
        rospy.loginfo("Shutting Down..")
		
		
if __name__=="__main__":
    rospy.init_node('voice_nav')
    
    try:
        voice_cmd_vel()
    except:
        pass

