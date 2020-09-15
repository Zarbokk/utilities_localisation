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

# publisher_waypoint = rospy.Publisher('/mavros/setpoint_position/local', PoseStamped, queue_size=10)
# publisher_waypoint = rospy.Publisher('/mavros/setpoint_raw/local', PositionTarget, queue_size=1)
publisher_waypoint = rospy.Publisher('mavros/setpoint_raw/attitude', AttitudeTarget, queue_size=1)
publisher_marker = rospy.Publisher('/infinity', MarkerArray, queue_size=1)
current_pos_number = 0
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

    return (np.asarray([range(0, N), np.flip(waypoints_x), np.flip(waypoints_y), np.flip(angle)]))


auftauchen = False
current_parameters = 0
R = 0.4
wanted_z_position = 0.5
distance_to_point = 0.8
thrust = 0.05
carrot = 1
roll_desired = 0
p = create_inf()
do_roll = False
just_changed = False


def rotation_matrix(angle):
    return np.asarray([[np.cos(angle), -np.sin(angle)], [np.sin(angle), np.cos(angle)]])


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
        tmp = np.matmul(R, np.asarray((x_max, y_max)))
        m = np.tan(np.arctan(current_waypoint[2]) - np.arctan2(y_max, x_max))
        x = np.linspace(0, tmp[0], number_of_points)

    if x_max < 0 and y_max > 0:
        angle_rotation_matrix = np.pi - np.arctan2(y_max, x_max)
        R = rotation_matrix(angle_rotation_matrix)
        R_return = rotation_matrix(-angle_rotation_matrix)
        tmp = np.matmul(R, np.asarray((x_max, y_max)))
        m = np.tan(np.arctan(current_waypoint[2]) - np.arctan2(y_max, x_max))
        x = np.linspace(tmp[0], 0, number_of_points)

    if x_max > 0 and y_max < 0:
        angle_rotation_matrix = -np.arctan2(y_max, x_max)
        R = rotation_matrix(angle_rotation_matrix)
        R_return = rotation_matrix(-angle_rotation_matrix)
        tmp = np.matmul(R, np.asarray((x_max, y_max)))
        m = np.tan(np.arctan(current_waypoint[2]) - np.arctan2(y_max, x_max))
        x = np.linspace(0, tmp[0], number_of_points)

    if x_max < 0 and y_max < 0:
        angle_rotation_matrix = np.pi - np.arctan2(y_max, x_max)
        R = rotation_matrix(angle_rotation_matrix)
        R_return = rotation_matrix(-angle_rotation_matrix)
        tmp = np.matmul(R, np.asarray((x_max, y_max)))
        m = np.tan(np.arctan(current_waypoint[2]) - np.arctan2(y_max, x_max))
        x = np.linspace(tmp[0], 0, number_of_points)

    a = m / tmp[0] / tmp[0]
    b = -m / tmp[0]
    y = a * x ** 3 + b * x ** 2

    current_path = np.matmul(R_return, np.asarray((x, y))) + np.asarray(
        [[current_position_boat[0]], [current_position_boat[1]]])
    point_to_be_controlled_on = current_path[:, number_of_points / 2]
    # print(point_to_be_controlled_on)
    # return current_waypoint
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
    for i in range(len(current_path[0])):
        marker = Marker()
        marker.header.frame_id = "global_tank"
        marker.id = i + p.shape[1]
        marker.type = marker.SPHERE
        marker.action = marker.ADD
        marker.scale.x = r  # r*2 of distance to camera from tag_14
        marker.scale.y = r
        marker.scale.z = r
        marker.color.g = 1
        marker.color.a = 1  # transparency
        marker.pose.orientation.w = 1.0
        marker.pose.position.x = current_path[0, i]  # x
        marker.pose.position.y = current_path[1, i]  # y
        marker.pose.position.z = wanted_z_position  # z
        markerArray.markers.append(marker)
    marker = Marker()
    marker.header.frame_id = "global_tank"
    marker.id = 200
    marker.type = marker.SPHERE
    marker.action = marker.ADD
    marker.scale.x = r  # r*2 of distance to camera from tag_14
    marker.scale.y = r
    marker.scale.z = r
    marker.color.g = 1
    marker.color.r = 1
    marker.color.a = 1  # transparency
    marker.pose.orientation.w = 1.0
    marker.pose.position.x = current_path[0, 10 / 2]  # x
    marker.pose.position.y = current_path[1, 10 / 2]  # y
    marker.pose.position.z = wanted_z_position  # z
    markerArray.markers.append(marker)

    publisher_marker.publish(markerArray)


def callback(msg):
    """"""
    global current_pos_number, N, R, p, rate, thrust, carrot, just_changed, do_roll
    current_pos = p[1:4, current_pos_number]
    # look if next waypoint should be loaded
    send_waypoint = AttitudeTarget()


    if np.sqrt(
            (msg.pose.position.x - current_pos[0]) ** 2 + (msg.pose.position.y - current_pos[1]) ** 2) < R:  # define R
        current_pos_number = current_pos_number + 1
        if current_pos_number > N - 1:
            current_pos_number = 0
        current_pos = p[1:4, current_pos_number]
    if current_pos_number == 18 and not just_changed:  # every time the the 24 waypoint is seen, then parameter can change
        change_parameter()
        just_changed = True
    if current_pos_number == 19:
        just_changed = False
    current_waypoint = pathplanning(current_pos, np.asarray([msg.pose.position.x, msg.pose.position.y]))
    rviz = True
    if rviz:
        visualization()

    rotation_body_frame = Quaternion(w=msg.pose.orientation.w,
                                     x=msg.pose.orientation.x,
                                     y=msg.pose.orientation.y,
                                     z=msg.pose.orientation.z)

    yaw_current, pitch_current, roll_current = rotation_body_frame.yaw_pitch_roll
    roll_current=-roll_current
    # yaw_current = -yaw_current
    # pitch_current = -pitch_current
    # roll_current = -((roll_current + 360 / 180.0 * np.pi) % (np.pi * 2) - 180 / 180.0 * np.pi)
    yaw_des = np.arctan2((current_waypoint[1] - msg.pose.position.y), (current_waypoint[0] - msg.pose.position.x))
    pitch_des = -np.arctan((wanted_z_position - msg.pose.position.z) / distance_to_point)
    roll_des = 0.0 / 180.0 * np.pi


    if auftauchen:
        pitch_des = np.pi / 2 - 0.1
        yaw_des = 0
        roll_des = 0

    send_waypoint.thrust = thrust * np.cos(yaw_current - yaw_des) * np.cos(roll_current - roll_des) * np.cos(
        pitch_current - pitch_des)
    if abs(yaw_current - yaw_des) > np.pi / 2 or abs(roll_current - roll_des) > np.pi / 2 or abs(
            pitch_current - pitch_des) > np.pi / 2:
        send_waypoint.thrust = 0

    if do_roll and current_pos_number > 55 and current_pos_number < 70:

        current_pos_number = 69
        roll_des = roll_current + np.pi / 2
        if roll_des > np.pi:
            roll_des = roll_des - np.pi * 2
        if roll_des > -np.pi / 3 and roll_des < 0:
            roll_des = 0
            do_roll = False
        send_waypoint.thrust = thrust*1.5
    # yaw_des = 0.0 / 180.0 * np.pi
    # pitch_des = 0.0 / 180.0 * np.pi

    qz_90n = Quaternion(
        axis=[0, 0, 1], angle=-(yaw_des - np.pi / 2)) * Quaternion(axis=[0, 1, 0], angle=-pitch_des) * Quaternion(
        axis=[1, 0, 0], angle=roll_des)


    send_waypoint.type_mask = 0
    send_waypoint.orientation.x = qz_90n.x
    send_waypoint.orientation.y = qz_90n.y
    send_waypoint.orientation.z = qz_90n.z
    send_waypoint.orientation.w = qz_90n.w
    # print(qz_90n.x,qz_90n.y,qz_90n.z,qz_90n.w)
    # 0.2 works
    send_waypoint.thrust = thrust * np.cos(yaw_current - yaw_des) * np.cos(
        pitch_current - pitch_des)
    if abs(yaw_current - yaw_des) > np.pi / 2  or abs(
            pitch_current - pitch_des) > np.pi / 2:
        send_waypoint.thrust = 0.0
    publisher_waypoint.publish(send_waypoint)
    rate.sleep()


def change_parameter():
    global current_parameters, R, thrust, distance_to_point, wanted_z_position, carrot, do_roll, auftauchen
    current_parameters = current_parameters + 1
    if current_parameters == 1 or current_parameters == 2 or current_parameters == 3 or current_parameters == 4 or current_parameters == 5 or current_parameters == 6 or current_parameters == 7:
        R = 0.4
        wanted_z_position = 0.5
        distance_to_point = 0.8
        thrust = 0.05
        do_roll = True

    if current_parameters == 8:
        R = 0.4
        wanted_z_position = 0.5
        distance_to_point = 0.8
        thrust = 0.10
        auftauchen = True
        do_roll = False
    return


def main():
    rospy.init_node('waypoint_send')
    global rate, R, wanted_z_position, distance_to_point, thrust, carrot, yaw
    rate = rospy.Rate(30)
    rospy.Subscriber("pose_px4", PoseStamped, callback, queue_size=1)
    rospy.spin()


if __name__ == '__main__':
    main()
