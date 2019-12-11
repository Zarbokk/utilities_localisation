from apriltag_ros.msg import AprilTagDetectionArray
from apriltag_ros.msg import AprilTagDetection
from gantry_msgs.msg import Gantry
import numpy as np

import rospy
import tf
from pyquaternion import Quaternion

# id,x,y,z,N,seen
number_of_tags=list()


def callback_april(msg):
    """"""
    global array_tags, number_of_tags
    num_meas = len(msg.detections)
    number_of_tags.append(num_meas)
    # print(array_tags)
    pass

def main():
    global mean_array, number_of_tags
    rospy.init_node('test_m_of_tags')
    rospy.Subscriber("/tag_detections", AprilTagDetectionArray, callback_april, queue_size=1)

    while not rospy.is_shutdown():
        pass
    np.savetxt("number_tags_70_half.csv", np.asarray(number_of_tags), delimiter=',')


if __name__ == '__main__':
    main()
