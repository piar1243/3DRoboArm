#!/usr/bin/env python

import random
from std_msgs.msg import Int32
import rospy
#import #time
from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32
from sensor_msgs.msg import JointState
from visualization_msgs.msg import InteractiveMarkerFeedback

from moveit_msgs.srv import GetPositionIK, GetPositionIKRequest

info = []

def callback(subscribedData):
    publisher_serial = rospy.Publisher('information', Int32, queue_size = 5)
    pub = rospy.Publisher('/rviz_moveit_motion_planning_display/robot_interaction_interactive_marker_topic/feedback', InteractiveMarkerFeedback, queue_size=1)
    #rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(1)
    #rospy.loginfo(subscribedData)
    #print(subscribedData)
    my_position = subscribedData
    print("active")
    my_position.pose.position.z = 0.48
    my_position.pose.position.x = -0.165
    my_position.pose.position.y = -0.4
    my_position.pose.orientation.z = -0.0112869925797
    my_position.pose.orientation.x = 0.710103690624
    my_position.pose.orientation.y = 0.704005360603


    quaternion = quaternion_from_euler(my_position.pose.orientation.x,
                                       my_position.pose.orientation.y,
                                       my_position.pose.orientation.z)[3]
    
    my_position.pose.orientation.w = -0.00136538851075#quaternion
    
    print(quaternion)
    #rospy.loginfo(my_position)
    pub.publish(my_position)
    #for i in "":
        
    #rospy.loginfo(intMessage)
    #publisher1.publish(info[cur_angle])
    rate.sleep()

def listener():
    
    rospy.Subscriber("/rviz_moveit_motion_planning_display/robot_interaction_interactive_marker_topic/feedback", InteractiveMarkerFeedback, callback)

#def publisher_coordinates():


def main():
    rospy.init_node('position_publisher', anonymous=True)
    listener()
    rospy.spin()

    
    

if __name__ == '__main__':
    main()