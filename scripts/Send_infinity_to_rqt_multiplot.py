#!/usr/bin/env python
import numpy as np

from pyquaternion import Quaternion
import rospy
# import tf

from geometry_msgs.msg import Pose, PoseArray, PoseStamped
from visualization_msgs.msg import Marker, MarkerArray
from mavros_msgs.msg import PositionTarget, AttitudeTarget
# from numpy import genfromtxt
# import os
# import matplotlib.pyplot as plt
import csv

# publisher_waypoint = rospy.Publisher('/mavros/setpoint_position/local', PoseStamped, queue_size=10)
# publisher_waypoint = rospy.Publisher('/mavros/setpoint_raw/local', PositionTarget, queue_size=1)
#publisher_waypoint = rospy.Publisher('inf', AttitudeTarget, queue_size=1)
#publisher_marker = rospy.Publisher('Sphere', MarkerArray, queue_size=1)
current_pos_number = 0
N = 100
rate = None


# set_mode_srv = rospy.ServiceProxy('mavros/set_mode', SetMode)
# res = set_mode_srv(0, " OFFBOARD")

def create_inf(dist_y=0.7, dist_x=0.9, r=0.5, dist_circle=1.2, N=100):
    while not N % 4 == 0:
        N = N + 1
    waypoints_x = np.zeros(0)
    waypoints_y = np.zeros(0)
    p1_x = dist_x
    p1_y = dist_y + r
    p2_x = dist_x + dist_circle
    p2_y = dist_y - r
    p3_x = dist_x + dist_circle
    p3_y = dist_y + r
    p4_x = dist_x
    p4_y = dist_y - r

    x = np.linspace(0, np.pi, num=N / 4)
    x_circle_r = dist_x - np.sin(x) * r
    y_circle_r = dist_y - np.cos(x) * r

    waypoints_x = np.append(waypoints_x, x_circle_r)
    waypoints_y = np.append(waypoints_y, y_circle_r)
    # at point 1
    waypoints_x = np.append(waypoints_x, np.linspace(p1_x, p2_x, N / 4))
    waypoints_y = np.append(waypoints_y, np.linspace(p1_y, p2_y, N / 4))
    # at point 2
    x_circle_l = dist_x + dist_circle + np.sin(x) * r
    y_circle_l = dist_y - np.cos(x) * r
    waypoints_x = np.append(waypoints_x, x_circle_l)
    waypoints_y = np.append(waypoints_y, y_circle_l)
    # at point 3
    waypoints_x = np.append(waypoints_x, np.linspace(p3_x, p4_x, N / 4))
    waypoints_y = np.append(waypoints_y, np.linspace(p3_y, p4_y, N / 4))
    # at point 4
    #print(waypoints_x.shape)
    #print(waypoints_y.shape)
    # plt.plot(waypoints_x, waypoints_y)
    # plt.plot(np.zeros(10), np.linspace(0, 2, 10))
    # plt.plot(np.ones(10) * 4.1, np.linspace(0, 2, 10))
    # plt.plot(np.linspace(0, 4.1, 10), np.zeros(10))
    # plt.plot(np.linspace(0, 4.1, 10), np.ones(10) * 2)
    # axes = plt.gca()
    # axes.set_xlim([-0.5, 4.5])
    # axes.set_ylim([-0.5, 4.5])
    # plt.show()
    waypoints_x = np.asarray(waypoints_x)
    waypoints_y = np.asarray(waypoints_y)
    #print(waypoints_x.shape)
    angle = np.zeros(N)
    for i in range(N):
        left_point = i - 1
        right_point = i + 1
        if i == 0:
            left_point = N - 1
            right_point = i + 1
        if i == N - 1:
            left_point = i - 1
            right_point = 0
        angle_trajectory = np.arctan2((-waypoints_y[left_point] + waypoints_y[right_point]),
                                      (-waypoints_x[left_point] + waypoints_x[right_point]))
        angle[i] = angle_trajectory
        #print(angle_trajectory)

    return (np.asarray([range(0, N), waypoints_x, waypoints_y, angle]))


def main():
    rospy.init_node('print_inf_rqt')
    R = 0.2
    wanted_z_position = 0
    distance_to_point = 0.5
    thrust = 0.2
    carrot = 1
    p = create_inf()
    rate = rospy.Rate(30)
    publisher_waypoint = rospy.Publisher('inf', PoseStamped, queue_size=1)
    #print(p.shape)
    for j in range(10):
        if not rospy.is_shutdown():
            for i in range(100):
                if  not rospy.is_shutdown():
                    msg = PoseStamped()
                    msg.pose.position.x = p[1, i]
                    msg.pose.position.y = p[2, i]
                    publisher_waypoint.publish(msg)
                    rate.sleep()



if __name__ == '__main__':
    main()
