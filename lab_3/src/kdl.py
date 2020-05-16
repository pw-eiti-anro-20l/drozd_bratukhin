#! /usr/bin/python

import rospy
import json
import os
import PyKDL as pykdl 
import math
from tf.transformations import *
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped

xaxis, yaxis, zaxis = (1, 0, 0), (0, 1, 0), (0, 0, 1)


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
		publish = False
		rospy.logwarn('Incorrect joint position[KDL]!')
		return

	chain = pykdl.Chain()
	frame = pykdl.Frame()
	angles = pykdl.JntArray(3)
	prev_d=0
	k=1
	prev_theta=0
	counter=0
	for i in params.keys():
		a, d, alpha, theta = params[i]
		a, d, alpha, theta = float(a), float(d), float(alpha), float(theta)
		joint = pykdl.Joint(pykdl.Joint.TransZ)
		if k!=1:
			fr = frame.DH(a, alpha, prev_d, prev_theta)
			chain.addSegment(pykdl.Segment(joint, fr))
		k=k+1
		prev_d=d
		prev_theta=theta
	
	chain.addSegment(pykdl.Segment(joint,frame.DH(0,0,d,theta)))	
	angles[0] = data.position[0]
	angles[1] = data.position[1]
	angles[2] = data.position[2]
	solver = pykdl.ChainFkSolverPos_recursive(chain)
	secFrame = pykdl.Frame()
	solver.JntToCart(angles,secFrame)
	quater = secFrame.M.GetQuaternion()
	pose = PoseStamped()
	pose.header.frame_id = 'base_link'
	pose.header.stamp = rospy.Time.now()
	pose.pose.position.x = secFrame.p[0]
	pose.pose.position.z = secFrame.p[2]
	pose.pose.position.y = secFrame.p[1]
	pose.pose.orientation.x = quater[0]
	pose.pose.orientation.y = quater[1]
	pose.pose.orientation.z = quater[2]		
	pose.pose.orientation.w = quater[3]
	pub.publish(pose)


if __name__ == '__main__':
  rospy.init_node('KDL_DKIN', anonymous=True)
  pub = rospy.Publisher('kdl_pose', PoseStamped, queue_size=10)
  rospy.Subscriber('joint_states', JointState, forward_kinematics)
  params = {}
  scope = {}
  print os.path.dirname(os.path.realpath(__file__))
  with open(os.path.dirname(os.path.realpath(__file__)) + '/../yaml/dhparams.json', 'r') as file:
    params = json.loads(file.read())
  with open(os.path.dirname(os.path.realpath(__file__)) + '/../yaml/restrictions.json', 'r') as file:
    scope = json.loads(file.read())
  rospy.spin()