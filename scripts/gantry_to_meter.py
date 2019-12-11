#!/usr/bin/env python
import numpy as np

from pyquaternion import Quaternion
import rospy
import tf

from geometry_msgs.msg import Pose, PoseArray, PoseStamped
from gantry_msgs.msg import Gantry

from apriltag_ros.msg import AprilTagDetectionArray
from apriltag_ros.msg import AprilTagDetection
from visualization_msgs.msg import Marker, MarkerArray
from numpy import genfromtxt
import os

publisher_position = rospy.Publisher('gantry_in_m', Gantry, queue_size=1)

def callback(msg):
    """"""
    meter_msg = Gantry()
    meter_msg.header = msg.header
    meter_msg.position.x = msg.position.x / 1000.0
    meter_msg.position.y = msg.position.y / 1000.0
    publisher_position.publish(meter_msg)
    # print(meter_msg)


def main():
    rospy.init_node('gantry_to_m')

    rospy.Subscriber("/gantry_position", Gantry, callback, queue_size=1)

    while not rospy.is_shutdown():
        pass


if __name__ == '__main__':
    main()
