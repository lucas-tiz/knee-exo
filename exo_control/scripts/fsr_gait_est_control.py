#!/usr/bin/env python
 
import rospy
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import MultiArrayDimension
import numpy as np

# control parameters
control_freq = 500 # (Hz) gait phase update frequency
fsr_threshold = 1.8 # (V) FSR threshold for triggering exo assistance
fsr_delay = 0.9 # (s) delay time after FSR trigger before next trigger enabled
num_strides_avg = 5 # number of previous stride durations to average over
num_strides_init = 10 # number of strides for initialization before controller is started
gait_phase_threshold = 90 # gait phase threshold for triggering exo assistance
assist_seq = [[0.00, 6.0], # inflate (psi) [time, pressure]
			  [0.05, 8.0],
			  [0.10, 10.0],
			  [0.15, 14.0], 
			  [0.20, 17.0],
			  [0.25, 20.0],
			  [0.30, 00.0]]
vent_seq = 	 [[0.00, 00.0], # vent
			  [1.00, -1.0]] # seal (turn off both solenoids)


class FsrController:
	''' perform preset actuator pressure sequence based on FSR load (voltage) '''
	def __init__(self, publisher_topic, subscriber_topic, control_freq, leg_side):
		self.num_pres_chan = 5 # number of pressure channels (fixed in microcontroller code)

		# set leg index
	   	if leg_side == 'right':
   			self.idx_leg_side = 0
		elif leg_side == 'left':
			self.idx_leg_side = 1

		# set up message
		self.msg_tx = Float32MultiArray() # create message 
		self.msg_tx.data = np.zeros(self.num_pres_chan+1, dtype=np.float32) # initialize with extra int for msg type
		self.msg_tx.data[0] = self.idx_leg_side # specify this is a pressure & corresponding leg
		self.msg_tx.layout.dim.append(MultiArrayDimension()) # add dimension to message
		self.msg_tx.layout.dim[0].size = self.num_pres_chan+1 # extra int for message type

		# set up variables
		self.control_freq = control_freq # gait phase update frequency
		self.assist_flag = 0
		self.t_fsr_trigger = np.array([rospy.get_time(),0]) # FSR trigger time history
		self.stride_dur = np.zeros(num_strides_avg) # stride duration history
		self.stride_dur_avg = 0 # stride duration average
		self.num_strides = 0 # number of strides taken
		self.num_stride_assist = 0 # stride for which last assist occurred

		# set up publisher and subscriber
		self.pub_tx = rospy.Publisher(publisher_topic, Float32MultiArray, queue_size=10) # publisher of data to transmit to MCU
		self.sub_tx = rospy.Subscriber(subscriber_topic, Float32MultiArray, queue_size=1, callback=self.fsrProcess) # subscriber to MCU data
		rospy.sleep(1) # required in order to not miss initial published data


	@staticmethod
	def expMovAvg(data):
		weights = np.exp(np.linspace(0, -1, num_strides_avg))
		weights /= weights.sum()
		return np.dot(data, weights)


	def fsrProcess(self, msg_rx):
		''' determine if FSR value is above trigger threshold, record stride '''
		fsr_value = msg_rx.data[self.idx_leg_side] # get FSR voltage value
		t = rospy.get_time() # get current time
		if (fsr_value >= fsr_threshold) and ((t - self.t_fsr_trigger[0]) >= fsr_delay): # if above threshold and past trigger delay
			self.t_fsr_trigger = np.roll(self.t_fsr_trigger, 1) # shift FSR trigger time history
			self.t_fsr_trigger[0] = t # update FSR trigger time
			self.stride_dur = np.roll(self.stride_dur, 1) # shift stride duration history
			self.stride_dur[0] = self.t_fsr_trigger[0] - self.t_fsr_trigger[1] # calculate latest stride duration
			self.stride_dur_avg = self.expMovAvg(self.stride_dur) # exponential moving average
			self.num_strides += 1 # increment number of strides
			print('strides: {:d}, average stride duration: {:0.3f} s'.format(self.num_strides, self.stride_dur_avg))

			if self.num_strides == num_strides_init: # if initialization strides finished
				print('assistance controller started')
				rospy.Timer(rospy.Duration(1.0/self.control_freq), callback=self.gaitPhaseUpdate) # start timer for gait phase update


	def gaitPhaseUpdate(self, event):
		''' check gait phase percentage, trigger pressure sequence if threshold reached '''
		gait_phase = (rospy.get_time() - self.t_fsr_trigger[0])/self.stride_dur_avg*100 # estimate gait phase
		if ((gait_phase >= gait_phase_threshold) and (self.assist_flag == 0) # if above threshold, not already assisting,
			and (self.num_stride_assist < self.num_strides)): # and current stride not yet assisted
			print('assistance triggered at: {:0.2f} % gait phase'.format(gait_phase))
			self.num_stride_assist = self.num_strides # update stride assist number to current stride
			self.assist_flag = 1 # set exo assistance flag
			self.runPresSequence(assist_seq) # run pressure assist sequence


	def runPresSequence(self, pres_seq):
		''' run pneumatic actuator pressure sequence '''
		t_start = rospy.get_time() # start time of pressure sequence
		ind_pres_seq = 0 # pressure sequence index
		while ind_pres_seq < len(pres_seq): # go through pressure sequence list
			t = rospy.get_time() - t_start
			if t >= pres_seq[ind_pres_seq][0]: # if at next sequence time
				self.msg_tx.data[1] = pres_seq[ind_pres_seq][1] # add pressure to message
				self.pub_tx.publish(self.msg_tx) # publish message
				print('pressure setpoint updated: ', pres_seq[ind_pres_seq][1], 'at time: ', t)
				ind_pres_seq = ind_pres_seq + 1 # increment pressure sequence index
		self.assist_flag = 0 # clear exo assistance flag



if __name__ == '__main__':  
	rospy.init_node('pres_control', anonymous=True) # initialize ROS node
   	leg_side = rospy.get_param('~leg_side') # get exo leg side
	fsr_controller = FsrController('mcu_tx', 'fsr', control_freq, leg_side) # create & start FSR controller

	def shutdown():
		fsr_controller.runPresSequence(vent_seq)
		rospy.sleep(2)
		print('vented!')

	rospy.on_shutdown(shutdown)
	rospy.spin()