#!/usr/bin/env python
import numpy as np
import rospkg
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

publisher_marker = rospy.Publisher('/infinity', MarkerArray, queue_size=1)
current_pos_number = 10
N = 100
rate = None
current_path = None


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
    waypoints_x = np.asarray(waypoints_x)
    waypoints_y = np.asarray(waypoints_y)
    # print(waypoints_x.shape)
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
        angle[i] = np.tan(angle_trajectory)
        # print(angle_trajectory)

    return (np.asarray([range(0, N), waypoints_x, waypoints_y, angle]))


R = 0.2
wanted_z_position = 0
distance_to_point = 0.5
thrust = 0.2
carrot = 1
p = create_inf()
def rotation_matrix(angle):
    return np.asarray([[np.cos(angle),-np.sin(angle)],[np.sin(angle),np.cos(angle)]])

def pathplanning(current_waypoint, current_position_boat):
    global current_path

    y_max = current_waypoint[1] - current_position_boat[1]
    x_max = current_waypoint[0] - current_position_boat[0]
    number_of_points = 10


    if False:
        if x_max > 0:
            x = np.linspace(0, x_max, number_of_points)
            c_cubic = -np.arctan2(-y_max, -x_max)
        else:
            x = np.linspace(x_max, 0, number_of_points)
            c_cubic = np.arctan2(-y_max, -x_max)
        d_cubic = 0
        a_cubic = (y_max + c_cubic / 2.0 * x_max - current_waypoint[2] / 2.0 * x_max - c_cubic * x_max) / (
                x_max ** 3 - 3.0 / 2.0 * x_max ** 3)
        b_cubic = (current_waypoint[2] - c_cubic - 3.0 * a_cubic * x_max ** 2) / 2.0 / x_max
        test = a_cubic * x ** 3 + b_cubic * x ** 2 + c_cubic * x + d_cubic
        stammfunktion = a_cubic * x_max ** 3 + b_cubic * x_max ** 2 + c_cubic * x_max + d_cubic
        ableitung = 3.0 * a_cubic * (x_max / 2.0) ** 2 + 2.0 * b_cubic * (x_max / 2.0) + c_cubic

        x = x - x_max + current_waypoint[0]
        test = test - y_max + current_waypoint[1]

    if x_max > 0 and y_max > 0:
        angle_rotation_matrix = -np.arctan2(y_max, x_max)
        R = rotation_matrix(angle_rotation_matrix)
        R_return = rotation_matrix(-angle_rotation_matrix)
        tmp = np.matmul(R , np.asarray((x_max, y_max)))
        m = np.tan(np.arctan(current_waypoint[2]) - np.arctan2(y_max, x_max))
        x = np.linspace(0, tmp[0], number_of_points)

    if x_max < 0 and y_max > 0:
        angle_rotation_matrix = np.pi - np.arctan2(y_max, x_max)
        R = rotation_matrix(angle_rotation_matrix)
        R_return = rotation_matrix(-angle_rotation_matrix)
        tmp = np.matmul(R , np.asarray((x_max, y_max)))
        m = np.tan(np.arctan(current_waypoint[2]) - np.arctan2(y_max, x_max))
        x = np.linspace(tmp[0], 0, number_of_points)

    if x_max > 0 and y_max < 0:
        angle_rotation_matrix = -np.arctan2(y_max, x_max)
        R = rotation_matrix(angle_rotation_matrix)
        R_return = rotation_matrix(-angle_rotation_matrix)
        tmp = np.matmul(R , np.asarray((x_max, y_max)))
        m = np.tan(np.arctan(current_waypoint[2]) - np.arctan2(y_max, x_max))
        x = np.linspace(0, tmp[0], number_of_points)

    if x_max < 0 and y_max < 0:
        angle_rotation_matrix = np.pi - np.arctan2(y_max, x_max)
        R = rotation_matrix(angle_rotation_matrix)
        R_return = rotation_matrix(-angle_rotation_matrix)
        tmp = np.matmul(R , np.asarray((x_max, y_max)))
        m = np.tan(np.arctan(current_waypoint[2]) - np.arctan2(y_max, x_max))
        x = np.linspace(tmp[0], 0, number_of_points)

    a = m / tmp[0] / tmp[0]
    b = -m / tmp[0]
    y = a * x **3 + b * x ** 2


    current_path = np.matmul(R_return ,np.asarray((x,y)))+np.asarray([[current_position_boat[0]],[current_position_boat[1]]])
    point_to_be_controlled_on=current_path[:,number_of_points/2]
    print(point_to_be_controlled_on)
    #return current_waypoint
    return point_to_be_controlled_on

def visualization():
    r = 0.05
    markerArray = MarkerArray()
    for i in range(p.shape[1]):
        if i == current_pos_number:
            marker = Marker()
            marker.header.frame_id = "global_tank"
            marker.id = i
            marker.type = marker.SPHERE
            marker.action = marker.ADD
            marker.scale.x = r * 2  # r*2 of distance to camera from tag_14
            marker.scale.y = r * 2
            marker.scale.z = r * 2
            marker.color.r = 1
            marker.color.a = 1  # transparency
            marker.pose.orientation.w = 1.0
            marker.pose.position.x = p[1, i]  # x
            marker.pose.position.y = p[2, i]  # y
            marker.pose.position.z = wanted_z_position  # z
            markerArray.markers.append(marker)
        else:
            marker = Marker()
            marker.header.frame_id = "global_tank"
            marker.id = i
            marker.type = marker.SPHERE
            marker.action = marker.ADD
            marker.scale.x = r  # r*2 of distance to camera from tag_14
            marker.scale.y = r
            marker.scale.z = r
            marker.color.g = 1
            marker.color.a = 1  # transparency
            marker.pose.orientation.w = 1.0
            marker.pose.position.x = p[1, i]  # x
            marker.pose.position.y = p[2, i]  # y
            marker.pose.position.z = wanted_z_position  # z
            markerArray.markers.append(marker)
    publisher_marker.publish(markerArray)




def main():
    rospy.init_node('waypoint_send')
    global rate, R, wanted_z_position, distance_to_point, thrust, carrot, yaw
    rate_2 = rospy.Rate(5)
    while not rospy.is_shutdown():
        # while 1:
        visualization()
        rate_2.sleep()


if __name__ == '__main__':
    main()
