#!/usr/bin/env python

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

SerialtopicName = 'angle_information'
delay = 0.1

class SimplePythonNode(object):
    def markers_cb(self, msg):
        self.desired_pos = PoseStamped()
        self.desired_pos.header = msg.header
        self.desired_pos.pose = msg.pose
        self.ik_compute()
        #rospy.Rate(1).sleep

    def ik_compute(self):
        req = GetPositionIKRequest()
        pubjointstates0 = Float64MultiArray()
        pubjointstates1 = Float64MultiArray()
        pubjointstates2 = Float64MultiArray()
        pubjointstates3 = Float64MultiArray()
        pubjointstates4 = Float64MultiArray()
        pubjointstates5 = Float64MultiArray()
        req.ik_request.group_name = "manipulator"
        req.ik_request.pose_stamped = self.desired_pos
        req.ik_request.robot_state.joint_state = self.joint_state
        req.ik_request.avoid_collisions = True
        resp = self.ik_srv.call(req)
        #print(resp.solution)
        try:
            #pubjointstates.position = resp.solution.joint_state.position
            currot1 = resp.solution.joint_state.position[0] * 57.2957795
            currot2 = resp.solution.joint_state.position[1] * 57.2957795
            currot3 = resp.solution.joint_state.position[2] * 57.2957795
            currot4 = resp.solution.joint_state.position[3] * 57.2957795
            currot5 = resp.solution.joint_state.position[4] * 57.2957795
            currot6 = resp.solution.joint_state.position[5] * 57.2957795
            #print(currot1)
            #pubjointstates.position[0] = resp.solution.joint_state.position[0] * 57.2957795
            #pubjointstates.position[1] = resp.solution.joint_state.position[1] * 57.2957795
            #pubjointstates.position[2] = resp.solution.joint_state.position[2] * 57.2957795
            #pubjointstates.position[3] = resp.solution.joint_state.position[3] * 57.2957795
            #pubjointstates.position[4] = resp.solution.joint_state.position[4] * 57.2957795
            #pubjointstates.position[5] = resp.solution.joint_state.position[5] * 57.2957795
            floatarray = np.array([currot1, currot2, currot3, currot4, currot5, currot6])
            floatarray.astype(np.float32)
            pubjointstates0.data = [currot1, 0]
            pubjointstates1.data = [currot2, 0]
            pubjointstates2.data = [currot3, 0]
            pubjointstates3.data = [currot4, 0]
            pubjointstates4.data = [currot5, 0]
            pubjointstates5.data = [currot6, 0]
            #pubjointstates.position[0] = 0.1231
            #rospy.loginfo(pubjointstates)
            #print(pubjointstates.position)
            #for item in pubjointstates.position:
            #    print(item)
            #pubjointstates.position[0] = pubjointstates.position[0]*57
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
            #rospy.Rate(1).sleep
            print(pubjointstates1.data)
        except:
            pass
        time.sleep(0.1)
        
        

    def __init__(self):
        self.joint_state_pub = rospy.Publisher(
            'joints',
            JointState,
            queue_size=1,
            latch=False)

        self.serial_publisher = rospy.Publisher('angle_information',
                                                 Float64MultiArray,
                                                   queue_size = 1)

        self.ik_srv = rospy.ServiceProxy("/compute_ik", GetPositionIK)

        self.marker_sub = rospy.Subscriber(
            '/rviz_moveit_motion_planning_display/robot_interaction_interactive_marker_topic/feedback',
            InteractiveMarkerFeedback,
            self.markers_cb, 
            queue_size = 1)


        self.joint_state_sub = rospy.Subscriber(
            'joint_states',
            JointState,
            self.joints_cb)

        
        self.test_sub = rospy.Subscriber(
            'test',
            Float32,
            self.float_cb)


        self.joint_timer = rospy.Timer(
            rospy.Duration(1.0),
            self.timer_cb)



    def timer_cb(self, _event):
        msg = JointState()
        now = rospy.Time.now()
        msg.header.stamp = now

        #rospy.loginfo(now)
        #rospy.loginfo('')
        #rospy.loginfo(now.to_sec())
        
        msg.position = [
            random.uniform(-1, 1),
            random.uniform(-1, 1),
            random.uniform(-1, 1),
            random.uniform(-1, 1),
            random.uniform(-1, 1),
            random.uniform(-1, 1)]

        self.joint_state_pub.publish(msg)


    def float_cb(self, msg):
        print(msg)


    def joints_cb(self, msg):
        # msg is a JointState message
        self.joint_state = msg


def main():
    """main"""
    rospy.init_node('inverse_kinematics_node')
    _simple_python_node = SimplePythonNode()
    rospy.spin()


if __name__ == "__main__":
    main()