#!/usr/bin/env python
import numpy as np

from pyquaternion import Quaternion
import rospy

from geometry_msgs.msg import Pose, PoseArray, PoseStamped, PointStamped
from apriltag_ros.msg import AprilTagDetectionArray
from sensor_msgs.msg import Imu

roll_imu = list()
pitch_imu = list()
yaw_imu = list()
x_acceleration_imu = list()
y_acceleration_imu = list()
z_acceleration_imu = list()
x_a_velocity_imu = list()
y_a_velocity_imu = list()
z_a_velocity_imu = list()
timestamp_imu = list()

x_april_ekf_pos = list()
y_april_ekf_pos = list()
z_april_ekf_pos = list()
yaw_april = list()
timestamp_april_ekf_pos = list()

x_apriltag = list()
y_apriltag = list()
z_apriltag = list()
roll_apriltag = list()
pitch_apriltag = list()
yaw_apriltag = list()
apriltag_number = list()
tag_id = list()
timestamp_april_tags_number = list()

rospy.init_node('get_data')

current_time = 0


def callback_imu_data(msg):
    global roll_imu, pitch_imu, yaw_imu, x_acceleration_imu, y_acceleration_imu, z_acceleration_imu, x_a_velocity_imu, y_a_velocity_imu, z_a_velocity_imu, timestamp_imu, current_time
    """"""
    rotation_body_frame = Quaternion(w=msg.orientation.w,
                                     x=msg.orientation.x,
                                     y=msg.orientation.y,
                                     z=msg.orientation.z)
    yaw, pitch, roll = rotation_body_frame.inverse.yaw_pitch_roll
    # print("first:", yaw * 180.0 / np.pi, pitch * 180.0 / np.pi, roll * 180.0 / np.pi)
    # yaw = (-yaw - 90 / 180.0 * np.pi + 360 / 180.0 * np.pi) % (np.pi * 2) - 180 / 180.0 * np.pi
    # yaw = -yaw
    # pitch = -pitch
    # roll = -((roll + 360 / 180.0 * np.pi) % (np.pi * 2) - 180 / 180.0 * np.pi)
    yaw = (yaw + np.pi / 2 + np.pi) % (np.pi * 2) - np.pi
    roll = -roll
    # print(yaw * 180.0 / np.pi, pitch * 180.0 / np.pi, roll * 180.0 / np.pi)

    roll_imu.append(roll)
    pitch_imu.append(pitch)
    yaw_imu.append(yaw)

    x_acceleration_imu.append(msg.linear_acceleration.x)
    y_acceleration_imu.append(msg.linear_acceleration.y)
    z_acceleration_imu.append(msg.linear_acceleration.z)

    x_a_velocity_imu.append(msg.angular_velocity.x)
    y_a_velocity_imu.append(msg.angular_velocity.y)
    z_a_velocity_imu.append(msg.angular_velocity.z)

    timestamp_imu.append((msg.header.stamp).to_sec() - current_time)


def callback_april_tag_ekf(msg):
    global x_april_ekf_pos, y_april_ekf_pos, z_april_ekf_pos, yaw_april, timestamp_april_ekf_pos
    """"""
    x_april_ekf_pos.append(msg.pose.position.x)
    y_april_ekf_pos.append(msg.pose.position.y)
    z_april_ekf_pos.append(msg.pose.position.z)

    rotation_body_frame = Quaternion(w=msg.pose.orientation.w,
                                     x=msg.pose.orientation.x,
                                     y=msg.pose.orientation.y,
                                     z=msg.pose.orientation.z)

    yaw, pitch, roll = rotation_body_frame.inverse.yaw_pitch_roll
    # yaw = (-yaw - 90 / 180.0 * np.pi + 360 / 180.0 * np.pi) % (np.pi * 2) - 180 / 180.0 * np.pi
    yaw = -yaw
    yaw_april.append(yaw)
    timestamp_april_ekf_pos.append((msg.header.stamp).to_sec() - current_time)
    return msg


def callback_tag_detections(msg):
    global x_apriltag, y_apriltag, z_apriltag, roll_apriltag, pitch_apriltag, yaw_apriltag, apriltag_number, tag_id, apriltag_number, timestamp_april_tags_number
    """"""
    num_meas = len(msg.detections)
    for i, tag in enumerate(msg.detections):
        tag_id.append(float(tag.id[0]))
        x_apriltag.append(tag.pose.pose.pose.position.x * 1.05)
        y_apriltag.append(tag.pose.pose.pose.position.y * 1.1)
        z_apriltag.append(tag.pose.pose.pose.position.z)
        tmpquat = Quaternion(w=tag.pose.pose.pose.orientation.w,
                             x=tag.pose.pose.pose.orientation.x,
                             y=tag.pose.pose.pose.orientation.y,
                             z=tag.pose.pose.pose.orientation.z)

        yaw_pitch_roll = tmpquat.yaw_pitch_roll
        roll_apriltag.append(float(yaw_pitch_roll[2]))
        pitch_apriltag.append(float(yaw_pitch_roll[1]))
        yaw_apriltag.append(float(yaw_pitch_roll[0]))

        apriltag_number.append(float(num_meas))
        timestamp_april_tags_number.append((msg.header.stamp).to_sec() - current_time)

    return msg


def myhook():
    global roll_imu, pitch_imu, yaw_imu, x_acceleration_imu, y_acceleration_imu, z_acceleration_imu, x_a_velocity_imu, y_a_velocity_imu, z_a_velocity_imu, timestamp_imu
    global x_april_ekf_pos, y_april_ekf_pos, z_april_ekf_pos, yaw_april, timestamp_april_ekf_pos
    global x_apriltag, y_apriltag, z_apriltag, roll_apriltag, pitch_apriltag, yaw_apriltag, tag_id, apriltag_number, timestamp_april_tags_number

    print("saving")
    np.savetxt("imu_data.csv",
               np.transpose([np.asarray(roll_imu, dtype=np.float64),
                             np.asarray(pitch_imu, dtype=np.float64),
                             np.asarray(yaw_imu, dtype=np.float64),
                             np.asarray(x_acceleration_imu, dtype=np.float64),
                             np.asarray(y_acceleration_imu, dtype=np.float64),
                             np.asarray(z_acceleration_imu, dtype=np.float64),
                             np.asarray(x_a_velocity_imu, dtype=np.float64),
                             np.asarray(y_a_velocity_imu, dtype=np.float64),
                             np.asarray(z_a_velocity_imu, dtype=np.float64),
                             np.asarray(timestamp_imu, dtype=np.float64)]), delimiter=",", fmt="%10.15f")

    np.savetxt("april_tag_ekf.csv",
               np.transpose(
                   [np.asarray(x_april_ekf_pos, dtype=np.float64),
                    np.asarray(y_april_ekf_pos, dtype=np.float64),
                    np.asarray(z_april_ekf_pos, dtype=np.float64),
                    np.asarray(yaw_april, dtype=np.float64),
                    np.asarray(timestamp_april_ekf_pos, dtype=np.float64)]),
               delimiter=",", fmt="%10.15f")

    np.savetxt("tag_detections.csv", np.transpose([np.asarray(x_apriltag, dtype=np.float64),
                                                   np.asarray(y_apriltag, dtype=np.float64),
                                                   np.asarray(z_apriltag, dtype=np.float64),
                                                   np.asarray(roll_apriltag, dtype=np.float64),
                                                   np.asarray(pitch_apriltag, dtype=np.float64),
                                                   np.asarray(yaw_apriltag, dtype=np.float64),
                                                   np.asarray(tag_id, dtype=np.float64),
                                                   np.asarray(apriltag_number, dtype=np.float64),
                                                   np.asarray(timestamp_april_tags_number, dtype=np.float64)]),
               delimiter=",", fmt="%10.15f")


def main():
    rospy.Subscriber("/mavros/imu/data", Imu, callback_imu_data, queue_size=1)
    rospy.Subscriber("/estimated_pose", PoseStamped, callback_april_tag_ekf, queue_size=1)
    rospy.Subscriber("/tag_detections", AprilTagDetectionArray, callback_tag_detections, queue_size=1)
    rospy.on_shutdown(myhook)
    rospy.spin()


if __name__ == '__main__':
    main()
