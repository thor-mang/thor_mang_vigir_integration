#!/usr/bin/env python

import roslib; roslib.load_manifest('thor_mang_fake_joint_states')
import rospy

from sensor_msgs.msg import JointState


class FakeJointStates(object):
    '''
    fake joint states
    '''


    def __init__(self):
        self._all_joints = ['head_pan', 'head_tilt', 'l_ankle_pitch', 'l_ankle_roll', 'l_elbow', 'l_f0_j0', 'l_f1_j0', 'l_hip_pitch', 'l_hip_roll', 'l_hip_yaw', 'l_knee', 'l_shoulder_pitch', 'l_shoulder_roll', 'l_shoulder_yaw', 'l_wrist_roll', 'l_wrist_yaw1', 'r_ankle_pitch', 'r_ankle_roll', 'r_elbow', 'r_f0_j0', 'r_f1_j0', 'r_hip_pitch', 'r_hip_roll', 'r_hip_yaw', 'r_knee', 'r_shoulder_pitch', 'r_shoulder_roll', 'r_shoulder_yaw', 'r_wrist_roll', 'r_wrist_yaw1', 'r_wrist_yaw2', 'waist_lidar', 'waist_pan', 'waist_tilt']
        
        self._sub = rospy.Subscriber("/thor_mang/joint_states_raw", self._joint_state_cb)
        self._pub = rospy.Publisher("/thor_mang/joint_states", JointState)

    def _joint_state_cb(self, joint_states):
        for joint in self._all_joints:
            if not joint in joint_states.name:
                joint_states.name.append(joint)
                joint_states.position.append(0)
                joint_states.velocity.append(0)
        self._pub.publish(joint_states)








        
