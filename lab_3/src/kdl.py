#! /usr/bin/python

import rospy
import json
import os
import PyKDL 
import math
from tf.transformations import *
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped

xaxis, yaxis, zaxis = (1, 0, 0), (0, 1, 0), (0, 0, 1)


def correct(pose):
    if pose.position[0] < rest['i1'][0] or pose.position[0] > rest['i1'][1]:
        return False

    if pose.position[1] < rest['i2'][0] or pose.position[1] > rest['i2'][1]:
        return False

    if pose.position[2] < rest['i3'][0] or pose.position[2] > rest['i3'][1]:
        return False

    return True


def forward_kinematics(data):
    chain = PyKDL.Chain()
    frame = PyKDL.Frame()
    k=1
    prev_d=0
    prev_th=0
    n_joints = len(params.keys())
    for i in params.keys():
        a, d, alpha, th = params[i]
        alpha, a, d, th = float(alpha), float(a), float(d), float(th)
        joint = PyKDL.Joint(PyKDL.Joint.TransZ)
	if k!=1:
            fr = frame.DH(a, alpha, prev_d, prev_th)
            segment = PyKDL.Segment(joint, fr)
            chain.addSegment(segment)
	k=k+1
	prev_d=d
	prev_th=th

    a, d, alpha, th = params["i3"]
    chain.addSegment(PyKDL.Segment(joint,frame.DH(0,0,d,th)))

    joints = PyKDL.JntArray(n_joints)
    for i in range(n_joints):
        min_joint, max_joint = rest["i"+str(i+1)]
        if min_joint <= data.position[i] <= max_joint:
            joints[i] = data.position[i]
        else:
            rospy.logwarn("Incorrect joint value")
            return

    fk=PyKDL.ChainFkSolverPos_recursive(chain)
    finalFrame=PyKDL.Frame()
    fk.JntToCart(joints,finalFrame)
    quaterions = finalFrame.M.GetQuaternion()

    pose = PoseStamped()
    pose.header.frame_id = 'base_link'
    pose.header.stamp = rospy.Time.now()
    pose.pose.position.x = finalFrame.p[0]
    pose.pose.position.y = finalFrame.p[1] 
    pose.pose.position.z = finalFrame.p[2]
    pose.pose.orientation.x = quaterions[0]
    pose.pose.orientation.y = quaterions[1]
    pose.pose.orientation.z = quaterions[2]
    pose.pose.orientation.w = quaterions[3]
    pub.publish(pose)


if __name__ == '__main__':
    rospy.init_node('KDL_KIN', anonymous=True)

    pub = rospy.Publisher('KdlAxes', PoseStamped, queue_size=10)
 

    rospy.Subscriber('joint_states', JointState, forward_kinematics)

    params = {}
    print os.path.dirname(os.path.realpath(__file__))
    with open(os.path.dirname(os.path.realpath(__file__)) + '/../yaml/dhparams.json', 'r') as file:
        params = json.loads(file.read())

    rest = {}
    with open(os.path.dirname(os.path.realpath(__file__)) + '/../yaml/restrictions.json', 'r') as file:
        rest = json.loads(file.read())

    rospy.spin()