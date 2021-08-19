#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import MultiArrayDimension
import numpy as np


num_ctrl_params = 3 # number of parameters per controller


if __name__ == '__main__':

	# set up node and publisher
	rospy.init_node('config_vals', anonymous=True) # initialize ROS node
   	pub_tx = rospy.Publisher('mcu_tx', Float32MultiArray, queue_size=10) # publisher of data to transmit to MCU
   	rospy.sleep(1) # required in order to not miss initial published data

	# set up message
	msg_tx = Float32MultiArray()
	msg_tx.data = np.zeros(num_ctrl_params+1, dtype=np.float32) # clear msg, extra float for msg type
	msg_tx.layout.dim.append(MultiArrayDimension())
	msg_tx.layout.dim[0].size = num_ctrl_params+1 # extra int for message type

	# loop through control parameters and update
	ctrllrs = ['muscle', 'bladder']
	rate = rospy.Rate(100) # publish at 100 Hz
	for idx, ctrllr in enumerate(ctrllrs):
		msg_tx.data[0] = (idx+2)
		msg_tx.data[1:] = [
			rospy.get_param(''.join(['/pres_ctrl_params/', ctrllr, '/deadband'])),
			rospy.get_param(''.join(['/pres_ctrl_params/', ctrllr, '/undershoot'])),
			rospy.get_param(''.join(['/pres_ctrl_params/', ctrllr, '/noise_threshold']))
		]
		pub_tx.publish(msg_tx) # publish message
		rate.sleep()

	rospy.loginfo('Control parameters updated.')