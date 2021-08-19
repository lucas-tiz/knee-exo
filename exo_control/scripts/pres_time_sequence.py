#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import MultiArrayDimension
import numpy as np

num_pres_chan = 2 # number of pressure channels

pres_seq = [[1.00, 1, 30.0], # (psi) pressure sequence [time, pressure]
			[2.00, 2, -1], # need last row to be 0 pressure to vent pneumatic actuators
			[3.00, 3, 0.0],
			[10.00, 4, -1]]

pres_seq = [[1.00, 10, 10], # (psi) pressure sequence [time, pressure]
			[2.00, -1, -1], # need last row to be 0 pressure to vent pneumatic actuators
			[3.00, 0,  0],
			[10.00,-1, -1]]

if __name__ == '__main__':
	# set up node and publisher
	rospy.init_node('pres_control', anonymous=True) # initialize ROS node
   	pub_tx = rospy.Publisher('mcu_tx', Float32MultiArray, queue_size=10) # publisher of data to transmit to MCU
   	rospy.sleep(1) # required in order to not miss initial published data

   	# get exo side
   	side = rospy.get_param('~leg_side')
   	if side == 'right':
   		msg_type = 0
	elif side == 'left':
		msg_type = 1

   	# set up message
	msg_tx = Float32MultiArray() # create message 
	msg_tx.data = np.zeros(num_pres_chan+1, dtype=np.float32) # initialize with extra int for msg type
	msg_tx.data[0] = msg_type # specify this is a pressure & corresponding leg
	msg_tx.layout.dim.append(MultiArrayDimension()) # add dimension to message
	msg_tx.layout.dim[0].size = num_pres_chan+1 # extra int for message type

	# run pressure sequence
	t_start = rospy.get_time() # start time of pressure sequence
	ind_pres_seq = 0 # pressure sequence index
	while ind_pres_seq < len(pres_seq): # go through pressure sequence list
		t = rospy.get_time() - t_start
		if t >= pres_seq[ind_pres_seq][0]: # if at next sequence time
			msg_tx.data[1:] = pres_seq[ind_pres_seq][1:] # add pressures to message
			pub_tx.publish(msg_tx) # publish message
			print('pressure setpoint updated: ', pres_seq[ind_pres_seq][1], 'at time: ', t)
			ind_pres_seq = ind_pres_seq + 1 # increment pressure sequence index
	print('  sequence completed')



