import numpy as np

from pyquaternion import Quaternion
import rospy
import tf

from geometry_msgs.msg import Pose, PoseArray, PoseStamped
from visualization_msgs.msg import Marker, MarkerArray
from mavros_msgs.msg import PositionTarget, AttitudeTarget
from numpy import genfromtxt
import os
import matplotlib.pyplot as plt
import csv

def smooth(y, box_pts):
    y=y[:,0]
    box = np.ones(box_pts)/box_pts
    y_smooth = np.convolve(y, box, mode='same')
    return y_smooth




data = None
data_path = 'number_tags_70_full.csv'
with open(data_path, 'r') as f:
    reader = csv.reader(f, delimiter=',')
    # get header from first row
    # get all the rows as a list
    data = list(reader)
    # transform data into numpy array
    data = np.array(data).astype(float)
    print(data.shape)


    # print(R)
smoothing=50
plt.plot(smooth(data,smoothing),'-o')
#plt.show()

data_path = 'number_tags_70_half.csv'
with open(data_path, 'r') as f:
    reader = csv.reader(f, delimiter=',')
    # get header from first row
    # get all the rows as a list
    data = list(reader)
    # transform data into numpy array
    data = np.array(data).astype(float)
    print(data.shape)

plt.plot(smooth(data,smoothing),'-o')
plt.show()

