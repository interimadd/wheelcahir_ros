#!/usr/bin/env python
# coding: UTF-8

import rospy
import time
import serial
from numpy import *
from multiprocessing import Process, Value, Array
from ultimate_seniorcar.msg import SeniorcarState

class CANUSB_Connecter:

	seniorcar_state = SeniorcarState()

	def __init__(self):
		rospy.init_node('canusb_connecter', anonymous=True)
		self.pub = rospy.Publisher('seniorcar_state',SeniorcarState, queue_size=1000)
		time.sleep(1)
		self.connect_with_canusb()
		self.pub.publish(self.seniorcar_state)
		print self.pub


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
		#t0A0 2 22 33�ϙC�������J�R�źţ�22 33 �β��֤�00 00 ����Ǥ���Фʤ�Ǥ�褤��
		#t0A1 8 00 00 accel 00 00 00 00 00 �ϥ��������_�Ȥ��ϕ���
		#t0A5 8 incliment 01 00 00 max_vel 00 00 1C other_commnad �ϥ��������_��������ϕ���.1C�ϸ��źŤ�����Ǥ���褦�ˤ��뤷�ʤ��Υե饰
		#other_command�������2byte���L���� 0 0 0 hone r_winker l_winker 0 light
		
		incliment = 11
		wait_time = 0

		while  not rospy.is_shutdown():

			if self.can_override_flag.value == 0:
				self.ser.write("t0A022233\r")
				self.ser.write("t0A58"+str(incliment)+"00000000000000\r")
			elif self.can_override_flag.value == 1:
				self.ser.write("t0A022233\r")
				print self.command_message(incliment)
				self.ser.write(self.command_message(incliment))
				#self.ser.write("t0A58"+str(incliment)+"01000000000F0F\r")
				if wait_time < 1:
					wait_time = wait_time + 0.05
				else:
					#self.ser.write("t0A180000330000000000\r")
					0
			time.sleep(0.05)

			incliment = incliment + 1
			if incliment > 90:
				incliment = 11

	# CANUSB���ͤ륷�ꥢ�륳�ޥ�������Ф�����
	def command_message(self,incliment):

		command = ""
		command += "t0A5" 
		command += "8"
		command += str(incliment)
		command += "010000"
		command += "00"	#max_vel
		command += "00"
		command += "1C"

		# 7byteĿ�˸�{���������Ф�2�M����ӛ
		hone   = int(self.seniorcar_command_array[8])
		r_wink = int(self.seniorcar_command_array[7])
		l_wink = int(self.seniorcar_command_array[6])
		light  = int(self.seniorcar_command_array[5])
		other_commands = "000"+str(hone)+str(r_wink)+str(l_wink)+"0"+str(light)
		other_commands = '%x' % int( other_commands, 2)
		
		if r_wink == 1:
			other_commands = "08"
		elif l_wink == 1:
			other_commands = "04"
		else:
			other_commands = "00"

		command += other_commands
		command += "\r"

		return command


	# ���Ť����Ť˥ץ�����֤��ƄI��
	def start_spin(self):

		self.can_override_flag = Value('i',0)
		self.seniorcar_state_array   = Array('d',(0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0))
		self.seniorcar_command_array = Array('d',(0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0))
		self.read_process  = Process( target = self.read_candata, args=(''))
		self.write_process = Process( target = self.write_candata,args=(''))

		try:
			self.read_process.start()
			self.write_process.start()
		except rospy.ROSInterruptException:
			pass

		rospy.Subscriber("seniorcar_command", SeniorcarState, self.command_recive)

		while not rospy.is_shutdown():
			self.update_vehicle_data()
			self.pub.publish(self.seniorcar_state)
			time.sleep(0.1)


	def command_recive(self,data):
		
		self.seniorcar_command_array[0] = data.accel_opening
		self.seniorcar_command_array[1] = data.max_velocity
		self.seniorcar_command_array[2] = data.steer_angle
		self.seniorcar_command_array[3] = data.vehicle_velocity * 3.6
		self.seniorcar_command_array[4] = data.direction_switch
		self.seniorcar_command_array[5] = data.light_signal
		self.seniorcar_command_array[6] = data.left_winker
		self.seniorcar_command_array[7] = data.right_winker
		self.seniorcar_command_array[8] = data.hone_signal


	def update_vehicle_data(self):

		self.seniorcar_state.accel_opening = self.seniorcar_state_array[0]
		self.seniorcar_state.max_velocity = self.seniorcar_state_array[1]
		self.seniorcar_state.steer_angle = self.seniorcar_state_array[2]
		self.seniorcar_state.vehicle_velocity = self.seniorcar_state_array[3] / 3.6  #  [km/h]����[m/s]�؉�Q
		self.seniorcar_state.direction_switch  = self.seniorcar_state_array[4]
		self.seniorcar_state.light_signal = self.seniorcar_state_array[5]
		self.seniorcar_state.left_winker = self.seniorcar_state_array[6]
		self.seniorcar_state.right_winker = self.seniorcar_state_array[7]
		self.seniorcar_state.hone_signal = self.seniorcar_state_array[8]

	
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
					self.update_can_information_array(can_data)
				elif can_id == "t010":		# �����ƥ��J�R�K�˥ե饰������
					if can_data[6:8] == "84":
						if self.can_override_flag.value == 0:
							print "change to interupt mode"
						self.can_override_flag.value = 1
					else:
						if self.can_override_flag.value == 1:
							print "change to manual mode"
						self.can_override_flag.value = 0
				elif can_id == "t0A0":
					print can_data
			else:
				line = line + serial_bit
				i = i + 1

		self.ser.close()


	def update_can_information_array(self , can_data):

		self.seniorcar_state_array[0] = float(int(can_data[:2] , 16)) / 127.0 # ���������_��
		self.seniorcar_state_array[1] = float(int(can_data[2:4] , 16)) / 100.0 * 4.0 + 2.0 #  ����ٶ�
		self.seniorcar_state_array[2] = float(int(can_data[6:8]   + can_data[4:6]  , 16)) / 10.0 - 90.0 # �ٶ�Ƕ�
		self.seniorcar_state_array[3] = float(int(can_data[10:12] + can_data[8:10] , 16)) / 100.0 # ܇�I�ٶ�

		# 1byte�Υǩ`������bit������
		byte7_data = int( can_data[14:16] ,16 )
		self.seniorcar_state_array[4] = byte7_data & 1
		self.seniorcar_state_array[5] = (byte7_data >> 1) & 1
		self.seniorcar_state_array[6] = (byte7_data >> 2) & 1
		self.seniorcar_state_array[7] = (byte7_data >> 3) & 1
		self.seniorcar_state_array[8] = (byte7_data >> 4) & 1


if __name__ == '__main__':

	connecter = CANUSB_Connecter()
	connecter.start_spin()