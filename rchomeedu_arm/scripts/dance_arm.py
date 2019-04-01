#!/usr/bin/env python

"""
    arm.py - move robot arm according to predefined gestures

"""

import rospy
from std_msgs.msg import String
from std_msgs.msg import Float64

class Loop:
    def __init__(self):
        rospy.on_shutdown(self.cleanup)

        rospy.Subscriber('/dance_arm', String, self.callback)

	# publish command message to joints/servos of arm
    	self.joint1 = rospy.Publisher('/waist_controller/command',Float64)
	self.joint2 = rospy.Publisher('/shoulder_controller/command',Float64)
    	self.joint3 = rospy.Publisher('/elbow_controller/command',Float64)
    	self.joint4 = rospy.Publisher('/wrist_controller/command',Float64)
	self.joint5 = rospy.Publisher('/hand_controller/command',Float64)
	self.pos1 = Float64()
    	self.pos2 = Float64()
    	self.pos3 = Float64()
    	self.pos4 = Float64()
    	self.pos5 = Float64()
	
	# Initial gesture of robot arm
	self.pos1 = 1.565
	self.pos2 = 2.102
	self.pos3 = -2.439
	self.pos4 = -1.294
	self.pos5 = 0.0
	self.joint1.publish(self.pos1)
	self.joint2.publish(self.pos2)
	self.joint3.publish(self.pos3)
	self.joint4.publish(self.pos4)
	self.joint5.publish(self.pos5)

    def callback(self, msg):
        print msg.data
        if msg.data == "dance arm":        
            count = 0
	    while (count < 1):

		# gesture 1
		self.pos1 = 0.559
		self.pos2 = 0.215
		self.pos3 = -1.508
		self.pos4 = -0.496
		self.pos5 = 0.0
		self.joint1.publish(self.pos1)
		self.joint2.publish(self.pos2)
		self.joint3.publish(self.pos3)
		self.joint4.publish(self.pos4)
		self.joint5.publish(self.pos5)
		rospy.sleep(2)
		
		# gesture 2
		self.pos1 = 0.565
		self.pos2 = -2.393
		self.pos3 = -0.639
		self.pos4 = 1.335
		self.pos5 = 0.0
		self.joint1.publish(self.pos1)
		self.joint2.publish(self.pos2)
		self.joint3.publish(self.pos3)
		self.joint4.publish(self.pos4)
		self.joint5.publish(self.pos5)
		rospy.sleep(3)

		# gesture 1
		self.pos1 = 0.559
		self.pos2 = 0.215
		self.pos3 = -1.508
		self.pos4 = -0.496
		self.pos5 = 0.0
		self.joint1.publish(self.pos1)
		self.joint2.publish(self.pos2)
		self.joint3.publish(self.pos3)
		self.joint4.publish(self.pos4)
		self.joint5.publish(self.pos5)
		rospy.sleep(2)

		# initial gesture
		self.pos1 = 0.565
		self.pos2 = 2.102
		self.pos3 = -2.439
		self.pos4 = -1.294
		self.pos5 = 0.0
		self.joint1.publish(self.pos1)
		self.joint2.publish(self.pos2)
		self.joint3.publish(self.pos3)
		self.joint4.publish(self.pos4)
		self.joint5.publish(self.pos5)
		rospy.sleep(3)

                count = count + 1

    def cleanup(self):
        rospy.loginfo("Shutting down robot arm....")

if __name__=="__main__":
    rospy.init_node('arm')
    try:
        Loop()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

