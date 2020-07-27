#!/usr/bin/env python
import numpy as np

from pyquaternion import Quaternion
import rospy

from geometry_msgs.msg import Pose, PoseArray, PoseStamped, PointStamped
from apriltag_ros.msg import AprilTagDetectionArray
from gantry_control_ros.msg import gantry

#ganrty test video from  1582926344.770884 to 1582926762.956707

x_gantry_pos = list()
y_gantry_pos = list()
z_gantry_pos = list()
timestamp_gantry_pos = list()

x_april_ekf_pos = list()
y_april_ekf_pos = list()
z_april_ekf_pos = list()
timestamp_april_ekf_pos = list()

apriltag_number = list()
timestamp_april_tags_number = list()

rospy.init_node('get_data')

current_time = 1582890800


def callback_gantry(msg):
    global x_gantry_pos, y_gantry_pos, z_gantry_pos, timestamp_gantry_pos
    """"""
    x_gantry_pos.append(msg.pos_gantry.x)
    y_gantry_pos.append(msg.pos_gantry.y)
    z_gantry_pos.append(msg.pos_gantry.z)
    timestamp_gantry_pos.append((msg.header.stamp).to_sec() - current_time)
    return msg


def callback_april_tag_ekf(msg):
    global x_april_ekf_pos, y_april_ekf_pos, z_april_ekf_pos, timestamp_april_ekf_pos
    """"""
    x_april_ekf_pos.append(msg.pose.position.x)
    y_april_ekf_pos.append(msg.pose.position.y)
    z_april_ekf_pos.append(msg.pose.position.z)
    timestamp_april_ekf_pos.append((msg.header.stamp).to_sec() - current_time)
    return msg


def callback_tag_detections(msg):
    global apriltag_number, timestamp_april_tags_number
    """"""
    num_meas = len(msg.detections)
    apriltag_number.append(num_meas)
    timestamp_april_tags_number.append((msg.header.stamp).to_sec() - current_time)
    return msg


def myhook():
    global x_gantry_pos, y_gantry_pos, z_gantry_pos, timestamp_gantry_pos, x_april_ekf_pos, y_april_ekf_pos, z_april_ekf_pos, timestamp_april_ekf_pos, apriltag_number, timestamp_april_tags_number
    print("saving")
    versuch = "0"
    np.savetxt("gantry_test_" + versuch + "_ekf.csv",
               np.transpose(
                   [np.asarray(x_april_ekf_pos, dtype=np.float32), np.asarray(y_april_ekf_pos, dtype=np.float32),
                    np.asarray(z_april_ekf_pos, dtype=np.float32),
                    np.asarray(timestamp_april_ekf_pos, dtype=np.float32)]),
               delimiter=",")

    np.savetxt("gantry_test_" + versuch + "_gantry_pos.csv",
               np.transpose(
                   [np.asarray(x_gantry_pos, dtype=np.float32), np.asarray(y_gantry_pos, dtype=np.float32),
                    np.asarray(z_gantry_pos, dtype=np.float32),
                    np.asarray(timestamp_gantry_pos, dtype=np.float32)]),
               delimiter=",")

    np.savetxt("gantry_test_" + versuch + "_number_of_tags.csv",
               np.transpose([np.asarray(apriltag_number, dtype=np.float32),
                             np.asarray(timestamp_april_tags_number,
                                        dtype=np.float32)]), delimiter=",")


def main():
    rospy.Subscriber("/mavros/vision_pose/pose_NED", PoseStamped, callback_april_tag_ekf, queue_size=1)
    rospy.Subscriber("/gantry/current_position", gantry, callback_gantry, queue_size=1)
    rospy.Subscriber("/tag_detections", AprilTagDetectionArray, callback_tag_detections, queue_size=1)
    rospy.on_shutdown(myhook)
    rospy.spin()


if __name__ == '__main__':
    main()
