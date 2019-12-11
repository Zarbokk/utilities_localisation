#!/usr/bin/env python
import numpy as np

from pyquaternion import Quaternion
import rospy
import tf

from geometry_msgs.msg import Pose, PoseArray, PoseStamped, PointStamped
from sensor_msgs.msg import Imu
from visualization_msgs.msg import Marker, MarkerArray
from mavros_msgs.msg import PositionTarget, AttitudeTarget
from numpy import genfromtxt
import os
import matplotlib.pyplot as plt
import csv

# change:
# abstand zu punkt damit gedaempft(nicht sehr viel maybe R=0.25
# distance to point verringern damit desired winkel groesser
publisher_rpy = rospy.Publisher("/mavros/imu/roll_pitch_yaw", PointStamped, queue_size=1)


def callback(msg):
    """"""
    # msg = PoseStamped()
    # msg=AttitudeTarget()
    # print("test")
    rotation_body_frame = Quaternion(w=msg.pose.orientation.w,
                                     x=msg.pose.orientation.x,
                                     y=msg.pose.orientation.y,
                                     z=msg.pose.orientation.z)
    yaw, pitch, roll = rotation_body_frame.yaw_pitch_roll
    print("first:",yaw* 180.0 / np.pi, pitch* 180.0 / np.pi, roll* 180.0 / np.pi)
    #yaw = (-yaw - 90 / 180.0 * np.pi + 360 / 180.0 * np.pi) % (np.pi * 2) - 180 / 180.0 * np.pi
    yaw=-yaw
    pitch = -pitch
    roll = (roll+ 360 / 180.0 * np.pi) % (np.pi * 2) - 180 / 180.0 * np.pi
    print(yaw * 180.0 / np.pi, pitch * 180.0 / np.pi, roll * 180.0 / np.pi)

    rpy_msg = PointStamped()
    rpy_msg.header = msg.header
    rpy_msg.point.x = roll * 180.0 / np.pi
    rpy_msg.point.y = pitch * 180.0 / np.pi
    rpy_msg.point.z = yaw * 180.0 / np.pi
    publisher_rpy.publish(rpy_msg)


def main():
    rospy.init_node('ypr_node_get')
    global rate, R, wanted_z_position, distance_to_point, thrust, carrot
    rate = rospy.Rate(30)
    rospy.Subscriber("/mavros/local_position/pose_NED", PoseStamped, callback, queue_size=1)
    # rospy.Subscriber("/mavros/setpoint_raw/attitude", AttitudeTarget, callback, queue_size=1)
    while not rospy.is_shutdown():
        pass


if __name__ == '__main__':
    main()
