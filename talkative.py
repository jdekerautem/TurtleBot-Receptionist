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
     

class talkative:
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
        
        self.soundhandle.playWave(self.wavepath + "/R2D2a.wav")
        rospy.sleep(1)
        
              
        # Subscribe to the /recognizer/output topic to receive voice commands.
        rospy.Subscriber('/recognizer/output', String, self.speechCb)
        
        # A mapping from keywords to commands.
        self.keywords_to_command = {'stop': ['stop', 'halt', 'abort', 'kill', 'panic', 'off', 'freeze', 'shut down', 'turn off'],
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
		self.soundhandle.say("Greetings! from your very own Charlie. How may I assist you?.", self.voice)
		rospy.sleep(3)
            
        elif command == 'bye':
        	self.soundhandle.say("Ok! See you soon. Have a Nice Day!. ", self.voice)
        	rospy.sleep(2)
        
        if command == 'help':
        	self.soundhandle.say("I am a receptionist robot. I can answer your questions about the faculty. or guide you through the faculty building!.", self.voice)
        	                    	
        if command == 'talk':
        	self.soundhandle.say("Oh! I am talking.. Have you seen my brother Wall E?.", self.voice)
        	        	
	if command == 'weather':
		self.soundhandle.say("Why don't you look out of the window?!. It is probably raining now!", self.voice)
        
	if command == 'supermarket':
		self.soundhandle.say("The nearest one is the TESCO, over the road, by the Reynold's building!. ", self.voice)
	
	if command == 'day':
		self.soundhandle.say("Today is tuesday!. Oh my God! it's still 3 days to the weekend", self.voice)

	if command == 'psychology':
		self.soundhandle.say("The psychology department is in the link building! You can get there! from the second floor of this building.!", self.voice)

	if command == 'teaching':
		self.soundhandle.say("The department is in the rolle building! You can find it on the north-west of the campus.!", self.voice)

	if command == 'engineering':
		self.soundhandle.say("That's right here! In the Smeaton's building! There are also brunel labs behind the library.!", self.voice)

	if command == 'biology':
		self.soundhandle.say("The department is in the Davy building! You can find it! across from the library over the plaza.!", self.voice)
	
	if command == 'maths':
		self.soundhandle.say("The department is in the babbage building! Leave via the western exit and take right across the road.!", self.voice)

	if command == 'geo':
		self.soundhandle.say("The department is in the Fitzroy building! You can find it north of the library beyond the trees!", self.voice)

	if command == 'marine':
		self.soundhandle.say("The marine department is in the reynolds building! And also the marine building where you can find the test tank, and the bridge simulators.!. It's across the road to the west of this building", self.voice)
	
	if command == 'art':
		self.soundhandle.say("The art faculty is in the scott building! You can find it directly south of the Smeaton's building.!", self.voice)
	
	if command == 'roland':
		self.soundhandle.say("That is in the roland levinsky building! You can find it directly south of the Smeaton's building.!", self.voice)
	
	if command == 'business':
		self.soundhandle.say("I believe that's in the cookworthy building! I have no memory of this place!", self.voice)

	if command == 'staff':
		self.soundhandle.say("You can find various members of staff in the Portland Square building!", self.voice)

	if command == 'sports':
		self.soundhandle.say("The sports facility is in the Nancy Astor building! You can find it on the northern edge of campus past Portland Square.!", self.voice)

	if command == 'robots':
		self.soundhandle.say("You can learn about robots here in Smeaton's building or in Portland Square. !", self.voice)	

	if command == 'cashpoint':
		self.soundhandle.say("There are cashpoints on the eastern exit of this building.! Don't forget your credit card!", self.voice)

	if command == 'visit':
		self.soundhandle.say("You can visit gin distillery or aquarium or go on a tour of The Sound.!", self.voice)
                    
        if command == 'name':
        	self.soundhandle.say("My name is CHARLIE.! CHArismatic Robot with Linguistic Interface . ", self.voice)
       	
       	if command == 'amazing':
       		self.soundhandle.say("Not as much as you are. But thank you so much.", self.voice)           
        
        if command == 'cafe':
        	self.soundhandle.say("There is one S U shop right opposite to the library. you can eat! or have a coffee there!", self.voice)      
       
        if command == 'wash':
        	self.soundhandle.say("The Gents toilet can be found on the second floor. and a ladies toilet on the third floor.", self.voice)
    
        if command == 'library':
        	self.soundhandle.say("As you will exit, out of eastern end of this building!. Take a left!. The building! in front of you! is the library!. ", self.voice)
        
        if command == 'labs':
        	self.soundhandle.say("The labs are on third floor of the Smeaton's building. Infact, we might be standing in one of those. ", self.voice)
    

    def cleanup(self):
        # When shutting down
	rospy.loginfo("Shutting Down...")
		
		
if __name__=="__main__":
    rospy.init_node('talkative')
    
    try:
        talkative()
    except:
        pass

