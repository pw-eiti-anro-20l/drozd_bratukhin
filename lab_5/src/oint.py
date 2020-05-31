#! /usr/bin/python

import rospy
from lab_4.srv import Oint
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
from nav_msgs.msg import Path
import math
from sys import stderr
from geometry_msgs.msg import PoseStamped

def linear_interpolation(start, end, t, exectime):
    return start + (end - start) * t / exectime

def polynomial_interpolation(start, end, t, exectime):
    a = -2*(end-start)/(exectime**3)
    b = 3*(end-start)/(exectime**2)
    return start + b*(t**2) + a*(t**3)

def set_interpolation_function(interpolation_type):
    if ( interpolation_type == "linear" ):
        interpolation_function = linear_interpolation
        return interpolation_function
    elif( interpolation_type == "polynomial" ):
        interpolation_function = polynomial_interpolation
        return interpolation_function
    else:
        return False

def check_validation(parametres):
    if( parametres.t <= 0):
        return False
    return True

def interpolate(data):
    global pose_pub
    global path_pub
    global prev_position
    global prev_q
    global path

    rate = rospy.Rate(50)
    if not check_validation(data):
        return "Enter valid time!"
    interpolation_function = set_interpolation_function(data.type)
    if not interpolation_function:
        return "Enter valid interpolation type: linear or polynomial"
    position = (data.x, data.y, data.z)
    q = (data.qx, data.qy, data.qz, data.qw)
    iterations_number = int(math.ceil(data.t * 50))
    t = 1.0 / 50.0
    for i in range(iterations_number):
    	x = interpolation_function(prev_position[0], position[0], t, data.t)
    	y = interpolation_function(prev_position[1], position[1], t, data.t)
    	z = interpolation_function(prev_position[2], position[2], t, data.t)
    	qx = interpolation_function(prev_q[0], q[0], t, data.t)
    	qy = interpolation_function(prev_q[1], q[1], t, data.t)
    	qz = interpolation_function(prev_q[2], q[2], t, data.t)
    	qw = interpolation_function(prev_q[3], q[3], t, data.t)
    	pose = PoseStamped()
    	pose.header.frame_id = "base_link"
    	pose.header.stamp = rospy.Time.now()
    	pose.pose.position.x = x
    	pose.pose.position.y = y
    	pose.pose.position.z = z
    	pose.pose.orientation.x = qx
    	pose.pose.orientation.y = qy
    	pose.pose.orientation.z = qz
    	pose.pose.orientation.w = qw
    	pose_pub.publish(pose)
    	path.header = pose.header
    	path.poses.append(pose)
    	path_pub.publish(path)
    	t = t + 1.0/50.0
    	rate.sleep()
    prev_position = position
    prev_q = q
    return "Interpolation completed succesfully!"


def oint():
    global pose_pub
    global path_pub
    global prev_position
    global prev_q
    global path
    path = Path()
    rospy.init_node('oint')
    prev_position = [ 0.0, 0.0, 0.0]
    prev_q = [ 0.0, 0.0, 0.0, 1.0]
    pose_pub = rospy.Publisher('oint_pose', PoseStamped, queue_size=10)
    path_pub = rospy.Publisher('oint_path', Path, queue_size=10)
    service = rospy.Service('oint_control_srv', Oint, interpolate)
    rospy.spin()

if __name__=="__main__":
    oint()