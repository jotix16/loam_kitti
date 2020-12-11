#!/usr/bin/python
# Autor> Clayder Gonzalez

import time
import rospy
import math
import numpy as np
import tf.transformations as tr

from nav_msgs.msg import Odometry

pub = None
ch_f_i = "odom"
h_f_i = "baselink"
position_covariance = 0.3
orientation_covariance= 0.2

R_2 = np.eye(4)
R_2[0,0] = -1

R_3 = np.eye(4)
t_imu_cam = np.array([1.1102242926728196, -0.31907194482240314, 0.7460658914035339]).reshape((3,-1))
t_bl_imu = np.array([-1.405, 0.32, 0.93]).reshape((3,-1))
R_3[0:3,0:3] = tr.quaternion_matrix([0.5042803291543856, -0.5003257925181871, 0.499074855710565676, -0.49628590285813])[0:3,0:3]
R_3[0:3,3:4] = t_bl_imu + t_imu_cam

R = np.matmul(R_3, R_2)



def odom_callback_loam(data):
	global pub
	global ch_f_i
	global h_f_i
	data.header.frame_id = h_f_i
	data.child_frame_id = ch_f_i

	position = np.matmul(R[0:3,0:3], np.array([data.pose.pose.position.x,
											data.pose.pose.position.y,
											data.pose.pose.position.z]).reshape((3,1)))
	quat  = tr.quaternion_from_matrix(np.matmul(R, tr.quaternion_matrix([data.pose.pose.orientation.x,
																		 data.pose.pose.orientation.y,
																		 data.pose.pose.orientation.z,
																		 data.pose.pose.orientation.w])))
	data.pose.pose.position.x = position[0]
	data.pose.pose.position.y = position[1]
	data.pose.pose.position.z = position[2]
	data.pose.pose.orientation.x = quat[0]
	data.pose.pose.orientation.y = quat[1]
	data.pose.pose.orientation.z = quat[2]
	data.pose.pose.orientation.w = quat[3]

	pub.publish(data)

def init():
	global pub
	pub = rospy.Publisher('/lidar/odom', Odometry, queue_size=10)
	rospy.init_node('remap_topics')
	ch_f_i = rospy.get_param('~child_frame_id', 'odom')
	h_f_i = rospy.get_param('~frame_id', 'baselink')
	rospy.Subscriber('/integrated_to_init', Odometry, odom_callback_loam)


	print( ch_f_i, h_f_i)
	rospy.spin()

if __name__ == '__main__':
	init()
