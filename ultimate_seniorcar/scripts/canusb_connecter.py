#!/usr/bin/env python
# coding: UTF-8

import rospy
import time
import serial
from numpy import *
from multiprocessing import Process, Value
from ultimate_seniorcar.msg import SeniorcarState

class CANUSB_Connecter:

	accel_opening = 0
	max_velocity = 0
	steer_angle = 0
	vehicle_velocity = 0
	direction_switch = 0
	light_signal = 0
	left_winker = 0
	right_winker = 0
	hone_signal = 0

	seniorcar_state = SeniorcarState()

	def __init__(self):
		rospy.init_node('canusb_connecter', anonymous=True)
		self.pub = rospy.Publisher('seniorcar_state',SeniorcarState, queue_size=10)
		time.sleep(1)
		self.connect_with_canusb()


	def connect_with_canusb(self):

		# シリアル通信開始
		port = rospy.get_param('canusb_port',"/dev/ttyUSB0")
		try:
			self.ser = serial.Serial(port,9600)
			self.ser.setDTR(False)
			time.sleep(0.5)
			self.ser.setDTR(True)
			print "start connection with canusb"
		except:
			print "cannot start connection with canusb"

		# CANUSBのCANポートを開くコマンド
		# 改行コマンド×複数回→通信速度設定→ポートオープン
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

		#A0への信号は0byteの場所にインクリメント(毎回変わる値)を入れてあげる必要がある
		#A0の6byteと7byteは同じにする
		
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
					#self.ser.write("t0A180000630000000000\r")
					0
			time.sleep(0.05)

			incliment = incliment + 1
			if incliment > 90:
				incliment = 11


	# 受信と送信にプロセスを分けて処理
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
				if can_id == "t01E":	# セニアカー走行データのID
					self.update_vehicle_data(can_data)
				elif can_id == "t010":		# システム認識終了フラグの受信
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

		self.seniorcar_state.accel_opening    = float(int(can_data[:2] , 16)) / 127.0
		self.seniorcar_state.max_velocity     = float(int(can_data[2:4] , 16)) / 100.0 * 4.0 + 2.0 
		self.seniorcar_state.steer_angle      = float(int(can_data[6:8]   + can_data[4:6]  , 16)) / 10.0 - 90.0
		self.seniorcar_state.vehicle_velocity = float(int(can_data[10:12] + can_data[8:10] , 16)) / 100.0

		# 1byteのデータからbit情報を抽出
		byte7_data = int( can_data[14:16] ,16 )
		self.seniorcar_state.direction_switch = byte7_data & 1
		self.seniorcar_state.light_signal     = (byte7_data >> 1) & 1
		self.seniorcar_state.left_winker      = (byte7_data >> 2) & 1
		self.seniorcar_state.right_winker     = (byte7_data >> 3) & 1
		self.seniorcar_state.hone_signal      = (byte7_data >> 4) & 1
		print "acc %-4.2f per ,max_vel %-4.2f km/h ,steer_angle %-4.2f deg ,vel %-4.2f km/h ,vel %-4.2f m/s" % (self.seniorcar_state.accel_opening,self.seniorcar_state.max_velocity,self.seniorcar_state.steer_angle,self.seniorcar_state.vehicle_velocity,self.seniorcar_state.vehicle_velocity/3.6)
		print "direction %d,light %d,left_winker %d,right_winker %d, hone_signal %d" % (self.seniorcar_state.direction_switch,self.seniorcar_state.light_signal,self.seniorcar_state.left_winker,self.seniorcar_state.right_winker,self.seniorcar_state.hone_signal)

		self.pub.publish(self.seniorcar_state)


if __name__ == '__main__':

	connecter = CANUSB_Connecter()
	connecter.start_spin()