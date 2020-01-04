#!/usr/bin/env python
import numpy as np

from pyquaternion import Quaternion
import rospy
import tf
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Pose, PoseArray, PoseStamped
from visualization_msgs.msg import Marker, MarkerArray
from numpy import genfromtxt
import os
import matplotlib.pyplot as plt

publisher_barometer = rospy.Publisher("/barometer", PoseStamped, queue_size=1)

rate = None

qx_180 = Quaternion(axis=[1, 0, 0], angle=np.pi)
qz_90p = Quaternion(axis=[0, 0, 1], angle=np.pi / 2)


def callback(msg):
    """"""
    global rate
    barometer_depth=-msg.pose[0].position.z+np.random.normal()/20#random number between 1/20+-
    NED = PoseStamped()
    NED.header.stamp = rospy.Time.now()
    NED.header.frame_id = 'global_tank'
    NED.pose.position.z = barometer_depth
    publisher_barometer.publish(NED)
    rate.sleep()


def main():
    rospy.init_node('gazebo_trouth_to_barometer')
    global rate
    rate = rospy.Rate(50)
    rospy.Subscriber("/gazebo/model_states", ModelStates, callback, queue_size=1)

    while not rospy.is_shutdown():
        pass


if __name__ == '__main__':
    main()
