#!/usr/bin/env python
import numpy as np

from pyquaternion import Quaternion
import rospy

from geometry_msgs.msg import Pose, PoseArray, PoseStamped, PointStamped
from apriltag_ros.msg import AprilTagDetectionArray

x_px4_pos = list()
y_px4_pos = list()
z_px4_pos = list()
timestamp_px4_pos = list()

x_april_ekf_pos = list()
y_april_ekf_pos = list()
z_april_ekf_pos = list()
timestamp_april_ekf_pos = list()

apriltag_number = list()
timestamp_april_tags_number = list()

rospy.init_node('get_data')

current_time = 1582890800

def callback_px4_pose(msg):
    global x_px4_pos,y_px4_pos,z_px4_pos,timestamp_px4_pos,current_time
    """"""
    x_px4_pos.append(msg.pose.position.x)
    y_px4_pos.append(msg.pose.position.y)
    z_px4_pos.append(msg.pose.position.z)
    timestamp_px4_pos.append((msg.header.stamp).to_sec()-current_time)

def callback_april_tag_ekf(msg):
    global x_april_ekf_pos, y_april_ekf_pos, z_april_ekf_pos, timestamp_april_ekf_pos
    """"""
    x_april_ekf_pos.append(msg.pose.position.x)
    y_april_ekf_pos.append(msg.pose.position.y)
    z_april_ekf_pos.append(msg.pose.position.z)
    timestamp_april_ekf_pos.append((msg.header.stamp).to_sec()-current_time)
    return msg

def callback_tag_detections(msg):
    global apriltag_number, timestamp_april_tags_number
    """"""
    num_meas = len(msg.detections)
    apriltag_number.append(num_meas)
    timestamp_april_tags_number.append((msg.header.stamp).to_sec()-current_time)
    return msg

def myhook():
    global x_px4_pos, y_px4_pos, z_px4_pos, timestamp_px4_pos,x_april_ekf_pos, y_april_ekf_pos, z_april_ekf_pos, timestamp_april_ekf_pos,apriltag_number, timestamp_april_tags_number
    print("saving")
    np.savetxt("px4_pos.csv", np.transpose([np.asarray(x_px4_pos, dtype=np.float32), np.asarray(y_px4_pos, dtype=np.float32),
                               np.asarray(z_px4_pos, dtype=np.float32),
                               np.asarray(timestamp_px4_pos, dtype=np.float32)]), delimiter=",")

    np.savetxt("april_tag_ekf.csv",
               np.transpose([np.asarray(x_april_ekf_pos, dtype=np.float32), np.asarray(y_april_ekf_pos, dtype=np.float32),
                np.asarray(z_april_ekf_pos, dtype=np.float32),np.asarray(timestamp_april_ekf_pos, dtype=np.float32)]),
               delimiter=",")

    np.savetxt("number_of_tags.csv", np.transpose([np.asarray(apriltag_number, dtype=np.float32),
                                      np.asarray(timestamp_april_tags_number, dtype=np.float32)]), delimiter=",")


def main():
    rospy.Subscriber("/mavros/local_position/pose_NED", PoseStamped, callback_px4_pose, queue_size=1)
    rospy.Subscriber("/estimated_pose", PoseStamped, callback_april_tag_ekf, queue_size=1)
    rospy.Subscriber("/tag_detections", AprilTagDetectionArray, callback_tag_detections, queue_size=1)
    rospy.on_shutdown(myhook)
    rospy.spin()


if __name__ == '__main__':
    main()
