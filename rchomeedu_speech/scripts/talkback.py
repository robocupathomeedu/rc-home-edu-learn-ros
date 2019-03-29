#!/usr/bin/env python

"""
    talkback.py - Version 1.1 2013-12-20
    
    Use the sound_play client to say back what is heard by the pocketsphinx recognizer.
    
"""

import rospy, os, sys
from std_msgs.msg import String
from sound_play.libsoundplay import SoundClient

class TalkBack:
    def __init__(self, script_path):
        rospy.init_node('talkback')

        rospy.on_shutdown(self.cleanup)
        
        # Create the sound client object
        self.soundhandle = SoundClient()
        
        # Wait a moment to let the client connect to the
        # sound_play server
        rospy.sleep(1)
        
        # Make sure any lingering sound_play processes are stopped.
        self.soundhandle.stopAll()
        
        # Announce that we are ready for input
        #self.soundhandle.playWave('say-beep.wav')
        #rospy.sleep(2)
        #self.soundhandle.say('Ready')
        
        rospy.loginfo("Say one of the navigation commands...")

        # Subscribe to the recognizer output and set the callback function
        rospy.Subscriber('/kws_data', String, self.talkback)
        
    def talkback(self, msg):
        # Print the recognized words on the screen
        rospy.loginfo(msg.data)
        
        # Speak the recognized words in the selected voice
        self.soundhandle.say("I heard " + msg.data, volume=0.01)
        rospy.sleep(5)

    def cleanup(self):
        self.soundhandle.stopAll()
        rospy.loginfo("Shutting down talkback node...")

if __name__=="__main__":
    try:
        TalkBack(sys.path[0])
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Talkback node terminated.")
