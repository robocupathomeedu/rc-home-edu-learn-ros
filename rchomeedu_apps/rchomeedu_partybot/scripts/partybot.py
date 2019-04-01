#!/usr/bin/env python

"""
    partybot.py - Version 0.2 2019-03-30
    
    A party robot to serve guests and entertainment.
    
"""

import rospy
from std_msgs.msg import String
from sound_play.libsoundplay import SoundClient
import sys
from subprocess import call
from geometry_msgs.msg import Twist
from math import radians
import os
from turtlebot_msgs.srv import SetFollowState

class PartyBot:
    def __init__(self, script_path):
        rospy.init_node('partybot')

        rospy.on_shutdown(self.cleanup)
        
        # Set the default TTS voice to use
        # self.voice = rospy.get_param("~voice", "voice_don_diphone")
        
        # Set the wave file path if used
        self.wavepath = rospy.get_param("~wavepath", script_path + "/../sounds")
        
        # Create the sound client object
        #self.soundhandle = SoundClient()
        self.soundhandle = SoundClient(blocking=True)
        
        # Wait a moment to let the client connect to the sound_play server
        rospy.sleep(1)
        
        # Make sure any lingering sound_play processes are stopped.
        self.soundhandle.stopAll()
        
        # Announce that we are ready for input
        self.soundhandle.playWave(self.wavepath + "/R2D2a.wav")
        #rospy.sleep(1)
        # self.soundhandle.say("Ready")
        
        rospy.loginfo("Ready, waiting for commands...")
	self.soundhandle.say('Hello, I am PartyBot. What can I do for you?')
	#rospy.sleep(2)

        # Subscribe to the recognizer output and set the callback function
        rospy.Subscriber('/lm_data', String, self.talkback)

	self.dance_arm = rospy.Publisher("dance_arm", String, queue_size=10)

        self.take_photo = rospy.Publisher("take_photo", String, queue_size=10)

	self.cmd_vel = rospy.Publisher('cmd_vel_mux/input/navi', Twist, queue_size=10)

    def control_follow(self, msg):
        rospy.wait_for_service('/turtlebot_follower/change_state')
        change_state = rospy.ServiceProxy('/turtlebot_follower/change_state', SetFollowState)
        response = change_state(msg)

    def talkback(self, msg):
        # Print the recognized words on the screen
        #msg.data=msg.data.lower()
        rospy.loginfo(msg.data)
        
        # Speak the recognized words in the selected voice
        # self.soundhandle.say(msg.data, self.voice)
        # call('rosrun sound_play say.py "montreal"', shell=True)
        # rospy.sleep(1)

	if msg.data.find('INTRODUCE-YOURSELF')>-1:
        	self.soundhandle.playWave(self.wavepath + "/R2D2a.wav")
        	#rospy.sleep(1)
		self.soundhandle.say("I heard you want me to introduce myself. I am PartyBot. I am a party robot to serve you and have fun.")
		#rospy.sleep(10) 
	elif msg.data.find('HOW-OLD-ARE-YOU')>-1:
        	self.soundhandle.playWave(self.wavepath + "/R2D2a.wav")
        	#rospy.sleep(1)
		self.soundhandle.say("I heard you ask about my age. I am five years old.")
		#rospy.sleep(5) 
	elif msg.data.find('FOLLOW-ME')>-1:
        	self.soundhandle.playWave(self.wavepath + "/R2D2a.wav")
        	#rospy.sleep(1)
		self.soundhandle.say("OK. I will start follow you.")
		#rospy.sleep(5)
                self.control_follow(1)
	elif msg.data.find('STOP-FOLLOW')>-1:
        	self.soundhandle.playWave(self.wavepath + "/R2D2a.wav")
        	#rospy.sleep(1)
		self.soundhandle.say("OK. I will stop follow you.")
		#rospy.sleep(5)
                self.control_follow(0)
	elif msg.data.find('ARE-YOU-FROM')>-1:
        	self.soundhandle.playWave(self.wavepath + "/R2D2a.wav")
        	#rospy.sleep(1)
		self.soundhandle.say("I heard you ask about my hometown. I am from China.")
		#rospy.sleep(5)
	elif msg.data.find('CAN-YOU-DO')>-1:
        	self.soundhandle.playWave(self.wavepath + "/R2D2a.wav")
        	#rospy.sleep(1)
		self.soundhandle.say("I heard you ask me what can I do? I am a home robot. I am good at singing and dancing. I tell funny jokes and I take great photos of people")
		#rospy.sleep(5)
	elif msg.data.find('TELL-A-FUNNY-JOKE')>-1:
        	self.soundhandle.playWave(self.wavepath + "/R2D2a.wav")
        	#rospy.sleep(1)
		self.soundhandle.say("You want to hear a joke? What is orange and sounds like a parrot? Erm, It is a carrot. Ha ha ha")
		#rospy.sleep(8)
	elif msg.data.find('SING-AND-DANCE')>-1:
        	self.soundhandle.playWave(self.wavepath + "/R2D2a.wav")
        	#rospy.sleep(1)
		self.soundhandle.say("You want me to sing and dance? sure. let me show you")
		#rospy.sleep(5)
                self.dance_arm.publish('dance arm')      	
		self.soundhandle.playWave(self.wavepath + "/swtheme.wav", blocking=False)
		#rospy.sleep(1) 
		# Dancing
		# create two different Twist() variables.  One for moving forward.  One for turning 45 degrees.
		# let's go forward at 0.2 m/s
		move_cmd = Twist()
		move_cmd.linear.x = 0.2
		# by default angular.z is 0 so setting this isn't required
	        #let's turn at 45 deg/s
        	turn_cmd = Twist()
        	turn_cmd.linear.x = 0
		turn_cmd.angular.z = radians(90); #45 deg/s in radians/s
        	turn_cmd2 = Twist()
        	turn_cmd2.linear.x = 0
		turn_cmd2.angular.z = radians(-90); #45 deg/s in radians/s
		# turn 90 degrees
		rospy.loginfo("Turning")
		for x in range(0,5):
			self.cmd_vel.publish(turn_cmd)
			# r.sleep()
			rospy.sleep(1)
			self.cmd_vel.publish(turn_cmd2)
			rospy.sleep(1) 
		rospy.sleep(6)
	elif msg.data.find('TAKE-A-PHOTO')>-1:
        	self.soundhandle.playWave(self.wavepath + "/R2D2a.wav")
        	#rospy.sleep(1)
		#call('rosrun image_view image_view image:=/camera_top/rgb/image_raw', shell=False)
		#rospy.sleep(1)
		self.soundhandle.say("You want to take a photo? Ok, get ready. One, two, three, say cheese")
                self.take_photo.publish('take photo')
                #call('rosrun rchomeedu_vision take_photo.py', shell=True)
		#rospy.sleep(6)
                #call('rosrun image_view image_saver image:=/camera_top/rgb/image_raw _save_all_image:=false _filename_format:=foo.jpg __name:=image_saver', shell=True)
		#call('rosservice call /image_saver/save', shell=True)
		#rospy.sleep(6)
	#else: self.soundhandle.say("Sorry, I cannot hear you clearly. Please say again.")
	else: rospy.sleep(3)
        
        # Uncomment to play one of the built-in sounds
        #rospy.sleep(2)
        #self.soundhandle.play(5)
        
        # Uncomment to play a wave file
        #rospy.sleep(2)
        #self.soundhandle.playWave(self.wavepath + "/R2D2a.wav")

    def cleanup(self):
        self.soundhandle.stopAll()
        rospy.loginfo("Shutting down partybot node...")

if __name__=="__main__":
    try:
        PartyBot(sys.path[0])
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Partybot node terminated.")
