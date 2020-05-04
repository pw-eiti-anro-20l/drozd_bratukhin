#! /usr/bin/python

import rospy
import json
import os
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


def kinematics(data):
    if not correct(data):
        rospy.logerr('Incorrect position! ')
        return
    T = translation_matrix((0, 0, 0))
    
    i=0
    for key in params:
        
        a,d,alpha,theta = params[key]
        matrixD= translation_matrix((0, 0, d+data.position[i]))
        matrixTheta = rotation_matrix(theta, zaxis)
        matrixA = translation_matrix((a, 0, 0))
        matrixAlpha = rotation_matrix(alpha, xaxis)

        TransMatrix = concatenate_matrices(matrixA,matrixAlpha,matrixTheta, matrixD)
        T=concatenate_matrices(T, TransMatrix)
        i=i+1
        
    
    x, y, z = translation_from_matrix(T)
    qx, qy, qz, qw = quaternion_from_matrix(T)
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
    rospy.init_node('NONKDL_KIN', anonymous=False)

    pub = rospy.Publisher('NoKdlAxes', PoseStamped, queue_size=10)

    rospy.Subscriber('joint_states', JointState, kinematics)

    params = {}
    print os.path.dirname(os.path.realpath(__file__))
    with open(os.path.dirname(os.path.realpath(__file__)) + '/../yaml/dhparams.json', 'r') as file:
        params = json.loads(file.read())


    rest = {}
    with open(os.path.dirname(os.path.realpath(__file__)) + '/../yaml/restrictions.json', 'r') as file:
        rest = json.loads(file.read())

    rospy.spin()