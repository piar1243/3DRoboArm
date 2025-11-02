#!/usr/bin/env python

# imports from ROS and MoveIt ------------------------------------------------------------------------------------------
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
# ----------------------------------------------------------------------------------------------------------------------

""" this script allows for position to be commanded by code and higher level autonomous programs to send their XYZ
and orientation coordinates to RVis, where they are then read by the inverse kinematics node and published to be
executed by the microcontroller and robotic arm movements"""


def callback(subscribedData):
    publisher_serial = rospy.Publisher('information', Int32, queue_size = 5)
    pub = rospy.Publisher('/rviz_moveit_motion_planning_display/robot_interaction_interactive_marker_topic/feedback',
                          InteractiveMarkerFeedback, queue_size=1)
    #rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(1)
    #rospy.loginfo(subscribedData)
    #print(subscribedData)
    my_position = subscribedData
    print("active")
    # position being published to the RVis robot that the Inverse Kinematics Node is subscribed to ---------------------
    my_position.pose.position.z = 0.48  # position is in meters
    my_position.pose.position.x = -0.165
    my_position.pose.position.y = -0.4
    my_position.pose.orientation.z = -0.0112869925797  # angles in degrees
    my_position.pose.orientation.x = 0.710103690624
    my_position.pose.orientation.y = 0.704005360603
    # ------------------------------------------------------------------------------------------------------------------


    quaternion = quaternion_from_euler(my_position.pose.orientation.x,
                                       my_position.pose.orientation.y,
                                       my_position.pose.orientation.z)[3]  # converts the angles into quaternions
    
    my_position.pose.orientation.w = -0.00136538851075  # quaternion
    
    print(quaternion)
    #rospy.loginfo(my_position)
    pub.publish(my_position)  # publishes the position states to the feedback
    #for i in "":
        
    #rospy.loginfo(intMessage)
    #publisher1.publish(info[cur_angle])
    rate.sleep()


def listener():  # listens for changes in the RViz interactive marker
    
    rospy.Subscriber("/rviz_moveit_motion_planning_display/robot_interaction_interactive_marker_topic/feedback",
                     InteractiveMarkerFeedback, callback)

#def publisher_coordinates():


def main():  # initializes the main position_publisher node
    rospy.init_node('position_publisher', anonymous=True)
    listener()
    rospy.spin()  # starts the program


if __name__ == '__main__':
    main()  # starts main
