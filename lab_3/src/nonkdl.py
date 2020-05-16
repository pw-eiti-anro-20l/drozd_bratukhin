#! /usr/bin/python

import rospy
import json
import os
from tf.transformations import *
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker

def checkScope(data):
    if data.position[0]<scope['i1'][0] or data.position[0]>scope['i1'][1]:
        return False
    if data.position[1]<scope['i2'][0] or data.position[1]>scope['i2'][1]:
        return False
    if data.position[2]<scope['i3'][0] or data.position[2]>scope['i3'][1]:
        return False

    return True

def forward_kinematics(data):

    if not checkScope(data):
        rospy.logwarn("Incorrect joint position[NON_KDL]!")
        return

    xaxis = (1,0,0)
    zaxis = (0,0,1)
    Tres=translation_matrix((0,0,0))
    count = 0
    for key in params.keys():
        a, d, alpha, theta = params[key]
        a, d, alpha, theta = float(a), float(d), float(alpha), float(theta)
        rotAlpha = rotation_matrix(alpha, xaxis)
        transA = translation_matrix((a, 0, 0))
        rotTheta = rotation_matrix(theta, zaxis)
        transD = translation_matrix((0, 0, d + data.position[count]))
        T = concatenate_matrices(rotAlpha, transA, rotTheta, transD)
        Tres = concatenate_matrices(Tres, T)
        count += 1

    x, y, z = translation_from_matrix(Tres)
    qx, qy, qz, qw = quaternion_from_matrix(Tres)

    pose = PoseStamped()
    pose.header.frame_id = 'base_link'
    pose.header.stamp = rospy.Time.now()
    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.position.z = z
    pose.pose.orientation.x = qx
    pose.pose.orientation.y = qy
    pose.pose.orientation.z = qz
    pose.pose.orientation.w = qw
    pub.publish(pose)

if __name__ == '__main__':
    rospy.init_node('NONKDL_DKIN', anonymous=True)
    pub = rospy.Publisher('nkdl_pose', PoseStamped, queue_size=10)
    rospy.Subscriber('joint_states', JointState, forward_kinematics)
    params = {}
    scope = {}
    with open(os.path.dirname(os.path.realpath(__file__)) + '/../yaml/dhparams.json', 'r') as file:
        params = json.loads(file.read())
        rospy.logwarn
    with open(os.path.dirname(os.path.realpath(__file__)) + '/../yaml/restrictions.json', 'r') as file:
        scope = json.loads(file.read())
    rospy.spin()