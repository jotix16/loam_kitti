#!/usr/bin/env python
# Autor> Clayder Gonzalez

import time
import rospy
import math
import numpy as np
import tf

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String

import matplotlib
import matplotlib.pyplot as plt
import math

trajectoryFile = None
tumtrajectoryFile = None
timeStamp = None
pitch = None

def odom_callback_loam(data):
	global trajectoryFile
	global tumtrajectoryFile
	global timeStamp
	global pitch
	global nr
	# print("Stamp of odom msg: ",data.header.stamp)
	# data.header.frame_id = h_f_i
	# data.child_frame_id = ch_f_i
	timeStampNum = data.header.stamp.secs + (data.header.stamp.nsecs * 10**(-9))
	timeStamp = str(timeStampNum)
	trajectoryFile.write(timeStamp + " " + str(data.pose.pose.position.z) + " " + str(data.pose.pose.position.x) + " " + str(pitch) + "\n")
	line = timeStamp + " " + str(data.pose.pose.position.x) + " " + str(data.pose.pose.position.y) + " " + str(data.pose.pose.position.z)
	line += " " + str(data.pose.pose.orientation.x) + " " + str(data.pose.pose.orientation.y) + " " + str(data.pose.pose.orientation.z) + " " + str(data.pose.pose.orientation.w) + "\n"
	tumtrajectoryFile.write(line)

def writeFile(file, tum_file):
	global trajectoryFile
	global tumtrajectoryFile
	# inicializa nodo ROS
	rospy.init_node('record_loam_odometry')
	path = rospy.get_param('~path', '')
	print("PATH:", path)
	trajectoryFile = open(file, 'w')
	tumtrajectoryFile = open(path + tum_file, 'w')

	rospy.Subscriber('/integrated_to_init', Odometry, odom_callback_loam)
	rospy.spin()
	trajectoryFile.close()
	tumtrajectoryFile.close()

if __name__ == '__main__':
	writeFile('LOAMTrajectory.txt', 'LOAMTrajectory.tum')
