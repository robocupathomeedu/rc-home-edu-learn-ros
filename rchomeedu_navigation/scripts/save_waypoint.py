#!/urs/bin/env python
# -*- encoding:UTF-8 -*-

import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

class get_point():

    def __init__(self):

        rospy.init_node("save_point")
        self.point_dataset = []
        self.point_num = []
        self.switch=True
        con = True
        print 'use keyboard to control your robot'
        print 'please input point_id if you want to exit please input stop'
        while con:
            print 'Get the waypoint : '
            str = raw_input()
            if self.switch == True:
                if str != 'stop':
                    self.switch = False
                    self.point_num.append(str)
                    self.save_waypoint()
                else:
                    con=False
                    self.write2txt()
            rospy.sleep(2)
                

    def save_waypoint(self):

        rospy.loginfo("save_point")
        amcl_sub = rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, self.amcl_callback)

    def amcl_callback(self, msg):

        my_point = MoveBaseGoal()
        my_point.target_pose.header.frame_id = '/map'
        my_point.target_pose.header.seq = msg.header.seq
        my_point.target_pose.header.stamp = msg.header.stamp
        my_point.target_pose.pose.position.x = msg.pose.pose.position.x
        my_point.target_pose.pose.position.y = msg.pose.pose.position.y
        my_point.target_pose.pose.position.z = msg.pose.pose.position.z
        my_point.target_pose.pose.orientation.x = msg.pose.pose.orientation.x
        my_point.target_pose.pose.orientation.y = msg.pose.pose.orientation.y
        my_point.target_pose.pose.orientation.z = msg.pose.pose.orientation.z
        my_point.target_pose.pose.orientation.w = msg.pose.pose.orientation.w
        self.point_dataset.append(my_point)
        rospy.loginfo("Point saved successfully!")
        self.switch = True

    def write2txt(self):
        
        f = open('/home/hts/ROS_test/src/nav_test/src/waypoints.txt','w')
        for point in range (len(self.point_dataset)):
            f.write(self.point_num[point])
            f.write(' ')
            f.write('position ')
            f.write('x '+str(self.point_dataset[point].target_pose.pose.position.x)+' ')
            f.write('y '+str(self.point_dataset[point].target_pose.pose.position.y)+' ')
            f.write('z '+str(self.point_dataset[point].target_pose.pose.position.z)+' ')
            f.write('orientation ')
            f.write('x '+str(self.point_dataset[point].target_pose.pose.orientation.x)+' ')
            f.write('y '+str(self.point_dataset[point].target_pose.pose.orientation.y)+' ')
            f.write('z '+str(self.point_dataset[point].target_pose.pose.orientation.z)+' ')
            f.write('w '+str(self.point_dataset[point].target_pose.pose.orientation.w)+' ')
            f.write('\n')
            

if __name__ == "__main__":
    
    try :
        get_point()
    except rospy.ROSInterruptException:
        print "------save_point_error!------"




