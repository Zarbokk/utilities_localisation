import numpy as np
import Tkinter
import rospy
from gantry_control_ros.msg import gantry
import tkFileDialog
import time

reached = None
gantry_pos = None


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




def move_to_position_ros(pub, current_target):
    msg_gantry = gantry()
    msg_gantry.header.stamp = rospy.Time.now()
    msg_gantry.pos_gantry.x = current_target[0]
    msg_gantry.pos_gantry.y = current_target[1]
    msg_gantry.pos_gantry.z = current_target[2]
    pub.publish(msg_gantry)


def follow_wp_and_take_measurements():
    waypoints_inf = create_inf()  # array=x,y angle
    rate = rospy.Rate(10)
    pub = rospy.Publisher('/gantry/position_des', gantry, queue_size=10)

    # start
    global reached, gantry_pos
    rospy.Subscriber("/gantry/current_position", gantry, callback)
    reached = False  # not robust solution
    for i in range(100):
        print("next wp = " + str(waypoints_inf[i, :]))
        current_target_m = waypoints_inf[i, :]
        current_target_m[2] = 0  # depth of gantry should change over each experiment.
        while np.linalg.norm(current_target_m - gantry_pos) > 0.1:
            move_to_position_ros(pub, current_target_m*1000)
            rate.sleep()

    return True


def callback(data):
    global reached, gantry_pos
    reached = data.reached
    gantry_pos = np.array([data.pos_gantry.x,
                           data.pos_gantry.y,
                           data.pos_gantry.z])


if __name__ == '__main__':
    rospy.init_node('Waypoint_Driver', anonymous=True)
    follow_wp_and_take_measurements()
