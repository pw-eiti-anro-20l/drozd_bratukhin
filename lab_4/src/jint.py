#!/usr/bin/env python

from time import sleep
from sys import stderr
from lab_4.srv import Jint
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
import math
import rospy

freq = 50.0

def linear_interpolation(start, end, t, exectime):
    return start + ((end-start)/exectime) * t

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
    elif( parametres.joint1 < -0.2 or parametres.joint1 > 0.15 or
        parametres.joint2 < -0.4 or parametres.joint2 > 0.0 or
        parametres.joint3 < -0.7 or parametres.joint3 > -0.5 ):
        return False
    return True

def interpolate(data):
    global pub
    global prev_joints
    if not check_validation(data):
        return "Enter valid data(time must be positive,joints restrictions: -0.2<=j1<=0.15,  -0.4<=j2<=0,  -0.7<=j3<=-0.5"
    interpolation_function = set_interpolation_function(data.type)
    if not interpolation_function:
        return "Enter valid interpolation type: linear or polynomial"
    rate = rospy.Rate(freq)

    jstate = JointState()
    jstate.name = ['base_link1', 'link1_link2', 'link2_link3']
    jstate.header = Header()
    end_joints = [data.joint1, data.joint2, data.joint3]
    iterations_number = int(math.ceil(data.t * freq))
    t = 1.0/freq
    for i in range(iterations_number):

        jstate.header.stamp = rospy.Time.now()
        joints = []
        joints.append(interpolation_function(prev_joints[0], end_joints[0], t, data.t))
        joints.append(interpolation_function(prev_joints[1], end_joints[1], t, data.t))
        joints.append(interpolation_function(prev_joints[2], end_joints[2], t, data.t))
        
        t = t + 1.0/freq

        jstate.position = joints
        pub.publish(jstate)
        rate.sleep()
    prev_joints=jstate.position
    return "Interpolation completed succesfully!"
    
def jint():
    global pub
    global prev_joints

    prev_joints=[0.0,0.0,-0.6]
    rospy.init_node('jint')
    pub = rospy.Publisher('/joint_states', JointState, queue_size = 10)
    service = rospy.Service('jint_control_srv', Jint, interpolate)
    rospy.spin()

if __name__ == "__main__":
    jint()