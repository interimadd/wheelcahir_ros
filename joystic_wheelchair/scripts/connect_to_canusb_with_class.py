#!/usr/bin/env python
# coding: UTF-8

import rospy
import time
import serial
from numpy import *
from multiprocessing import Process, Value

class CANUSB_Conecter:

	accel_opening = 0
	max_velocity = 0
	steer_angle = 0
	vehicle_velocity = 0
	direction_switch = 0
	light_signal = 0
	left_winker = 0
	right_winker = 0
	hone_signal = 0


	def __init__(self):
		rospy.init_node('coneect_to_canusb', anonymous=True)
		time.sleep(1)
		self.connect_with_canusb()


	def connect_with_canusb(self):

		# ���ꥢ��ͨ���_ʼ
		port = rospy.get_param('canusb_port',"/dev/ttyUSB0")
		try:
			self.ser = serial.Serial(port,9600)
			self.ser.setDTR(False)
			time.sleep(0.5)
			self.ser.setDTR(True)
			print "start connection with canusb"
		except:
			print "cannot start connection with canusb"

		# CANUSB��CAN�ݩ`�Ȥ��_�����ޥ��
		# ���Х��ޥ�ɡ��}���ء�ͨ���ٶ��O�����ݩ`�ȥ��`�ץ�
		time.sleep(0.5)
		self.ser.write("\r")
		self.ser.write("\r")
		time.sleep(0.5)
		self.ser.write("C\r")
		time.sleep(0.5)
		self.ser.write("S6\r")
		time.sleep(0.5)
		self.ser.write("O\r")

		print "can open"


	def write_candata(self):

		#A0�ؤ��źŤ�0byte�Έ����˥��󥯥����(���؉��낎)�����Ƥ������Ҫ������
		#A0��6byte��7byte��ͬ���ˤ���
		
		incliment = 11
		wait_time = 0

		while  not rospy.is_shutdown():

			if self.can_override_flag.value == 0:
				self.ser.write("t0A022233\r")
				self.ser.write("t0A58"+str(incliment)+"00000000000000\r")
			elif self.can_override_flag.value == 1:
				self.ser.write("t0A022233\r")
				self.ser.write("t0A58"+str(incliment)+"01000000000000\r")
				if wait_time < 1:
					wait_time = wait_time + 0.05
				else:
					self.ser.write("t0A180000630000000000\r")
			time.sleep(0.05)

			incliment = incliment + 1
			if incliment > 90:
				incliment = 11


	# ���Ť����Ť˥ץ�����֤��ƄI��
	def start_spin(self):

		self.can_override_flag = Value('i',0)
		self.read_process  = Process( target = self.read_candata, args=(''))
		self.write_process = Process( target = self.write_candata,args=(''))

		try:
			self.read_process.start()
			self.write_process.start()
		except rospy.ROSInterruptException:
			pass

	
	def read_candata(self):

		line = ""
		can_id   = ""
		can_data = ""

		i = 0

		while  not rospy.is_shutdown():
			serial_bit = self.ser.read()
			if serial_bit == "\r" and i > 10:
				can_id   = line[:4]
				#can_bit  = line[4]
				can_data = line[5:]
				line = ""
				i = 0
				if can_id == "t01E":	# ���˥����`���Хǩ`����ID
					self.update_vehicle_data(can_data)
				elif can_id == "t010":		# �����ƥ��J�R�K�˥ե饰������
					if can_data[6:8] == "84":
						self.can_override_flag.value = 1
						print "change to interupt mode"
					else:
						self.can_override_flag.value = 0
						print "change to manual mode"
				elif can_id == "t0A0":
					print can_data
			else:
				line = line + serial_bit
				i = i + 1

		ser.close()


	def update_vehicle_data(self , can_data):

		self.accel_opening    = float(int(can_data[:2] , 16)) / 127.0
		self.max_velocity     = float(int(can_data[2:4] , 16)) / 100.0 * 4.0 + 2.0 
		self.steer_angle      = float(int(can_data[6:8]   + can_data[4:6]  , 16)) / 10.0 - 90.0
		self.vehicle_velocity = float(int(can_data[10:12] + can_data[8:10] , 16)) / 100.0

		# 1byte�Υǩ`������bit������
		byte7_data = int( can_data[14:16] ,16 )
		self.direction_switch = byte7_data & 1
		self.light_signal     = (byte7_data >> 1) & 1
		self.left_winker      = (byte7_data >> 2) & 1
		self.right_winker     = (byte7_data >> 3) & 1
		self.hone_signal      = (byte7_data >> 4) & 1
		print "acc %-4.2f per ,max_vel %-4.2f km/h ,steer_angle %-4.2f deg ,vel %-4.2f km/h ,vel %-4.2f m/s" % (self.accel_opening,self.max_velocity,self.steer_angle,self.vehicle_velocity,self.vehicle_velocity/3.6)
		print "direction %d,light %d,left_winker %d,right_winker %d, hone_signal %d" % (self.direction_switch,self.light_signal,self.left_winker,self.right_winker,self.hone_signal)


if __name__ == '__main__':

	connecter = CANUSB_Conecter()
	connecter.start_spin()