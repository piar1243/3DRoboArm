#!/usr/bin/env python

# imports from ROS and MoveIt ------------------------------------------------------------------------------------------
import random
from std_msgs.msg import Int32
import rospy
import numpy as np
import time
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32
from sensor_msgs.msg import JointState
from visualization_msgs.msg import InteractiveMarkerFeedback
from moveit_msgs.srv import GetPositionIK, GetPositionIKRequest
# ----------------------------------------------------------------------------------------------------------------------

SerialtopicName = 'angle_information'  # name of the serial information
delay = 0.1  # delay between updates in publishing joint angle position through ROSSerial to arduino


class SimplePythonNode(object):  # makes node
    def markers_cb(self, msg):
        self.desired_pos = PoseStamped()  # the desired position
        self.desired_pos.header = msg.header  # the head and the pose
        self.desired_pos.pose = msg.pose
        self.ik_compute()  # computes the inverse kinematic position
        #rospy.Rate(1).sleep

    def ik_compute(self):  # gets the IK angle information
        req = GetPositionIKRequest()  # requests the IK positions
        pubjointstates0 = Float64MultiArray()  # defines publishing joint states that will be sent to microcontroller
        pubjointstates1 = Float64MultiArray()
        pubjointstates2 = Float64MultiArray()
        pubjointstates3 = Float64MultiArray()
        pubjointstates4 = Float64MultiArray()
        pubjointstates5 = Float64MultiArray()
        req.ik_request.group_name = "manipulator"  # gets the end effector
        req.ik_request.pose_stamped = self.desired_pos  # gets the desired position
        req.ik_request.robot_state.joint_state = self.joint_state  # gets the joint state for RViz
        req.ik_request.avoid_collisions = True  # RViz setup the joint collisions to ensure angles don't get buggy
        resp = self.ik_srv.call(req)  # calls the IK service and waits for response
        #print(resp.solution)  # debugging
        try:
            #pubjointstates.position = resp.solution.joint_state.position
            currot1 = resp.solution.joint_state.position[0] * 57.2957795  # gets the joint angles from radians
            currot2 = resp.solution.joint_state.position[1] * 57.2957795
            currot3 = resp.solution.joint_state.position[2] * 57.2957795
            currot4 = resp.solution.joint_state.position[3] * 57.2957795
            currot5 = resp.solution.joint_state.position[4] * 57.2957795
            currot6 = resp.solution.joint_state.position[5] * 57.2957795
            #print(currot1)

            floatarray = np.array([currot1, currot2, currot3, currot4, currot5, currot6])  # defines array of angles
            floatarray.astype(np.float32)  # makes the data float 32 to be able to send over ROSSerial
            pubjointstates0.data = [currot1, 0]  # defines each joint state as data list for ROSSerial debugging
            pubjointstates1.data = [currot2, 0]
            pubjointstates2.data = [currot3, 0]
            pubjointstates3.data = [currot4, 0]
            pubjointstates4.data = [currot5, 0]
            pubjointstates5.data = [currot6, 0]
            #pubjointstates.position[0] = 0.1231  # debugging
            #rospy.loginfo(pubjointstates)
            #print(pubjointstates.position)
            #for item in pubjointstates.position:
            #    print(item)
            #pubjointstates.position[0] = pubjointstates.position[0]*57
            # publishes angles to arduino through ROSSerial ------------------------------------------------------------
            time.sleep(delay)
            self.serial_publisher.publish(pubjointstates0)
            time.sleep(delay)
            self.serial_publisher.publish(pubjointstates1)
            time.sleep(delay)
            self.serial_publisher.publish(pubjointstates2)
            time.sleep(delay)
            self.serial_publisher.publish(pubjointstates3)
            time.sleep(delay)
            self.serial_publisher.publish(pubjointstates4)
            time.sleep(delay)
            self.serial_publisher.publish(pubjointstates5)
            time.sleep(delay)
            # ----------------------------------------------------------------------------------------------------------
            #rospy.Rate(1).sleep
            print(pubjointstates1.data)
        except:
            pass
        time.sleep(0.1)  # time.sleep ensure the serial doesn't get overwhelmed

    def __init__(self):  # initialization
        self.joint_state_pub = rospy.Publisher(
            'joints',
            JointState,
            queue_size=1,
            latch=False)  # defines the joint state publisher

        self.serial_publisher = rospy.Publisher('angle_information',
                                                 Float64MultiArray,
                                                   queue_size = 1)  # defines the ROSSerial publisher

        self.ik_srv = rospy.ServiceProxy("/compute_ik", GetPositionIK)  # gets the IK service from MoveIt

        self.marker_sub = rospy.Subscriber(
            '/rviz_moveit_motion_planning_display/robot_interaction_interactive_marker_topic/feedback',
            InteractiveMarkerFeedback,
            self.markers_cb, 
            queue_size = 1)  # subscribes to the feedback python node that gets information from RVis

        self.joint_state_sub = rospy.Subscriber(
            'joint_states',
            JointState,
            self.joints_cb)  # defines the joint state publisher as a subscription to the joint states

        self.test_sub = rospy.Subscriber(
            'test',
            Float32,
            self.float_cb)  # defines the test subscribe as the float 32 output

        self.joint_timer = rospy.Timer(
            rospy.Duration(1.0),
            self.timer_cb)  # sets the timer to call back every one second

    def timer_cb(self, _event):
        msg = JointState()  # sets the message as the joint state
        now = rospy.Time.now()  # gets the now time
        msg.header.stamp = now  # stamps the joint state with time

        #rospy.loginfo(now)
        #rospy.loginfo('')
        #rospy.loginfo(now.to_sec())
        
        msg.position = [
            random.uniform(-1, 1),
            random.uniform(-1, 1),
            random.uniform(-1, 1),
            random.uniform(-1, 1),
            random.uniform(-1, 1),
            random.uniform(-1, 1)]  # creates random arbitrary position set to be generated over

        self.joint_state_pub.publish(msg)  # publishes the arbitrary joint states to joint state publisher

    def float_cb(self, msg):  # prints the call back
        print(msg)

    def joints_cb(self, msg):  # assigns the joint states on callback
        # msg is a JointState message
        self.joint_state = msg  # the joint state is the message

def main():
    """main"""
    rospy.init_node('inverse_kinematics_node')  # initializes this python scrip[t as the inverse kinematics node
    _simple_python_node = SimplePythonNode()  # makes it a python node
    rospy.spin()  # start the program


if __name__ == "__main__":
    main()  # starts the program
