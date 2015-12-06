#!/usr/bin/env python
# coding: UTF-8

########
#計測結果：4600mmの直進移動に対して4736mm移動したという出力
##       : 2回転(2*3.14*2=12.56)に対して13.06の出力

import rospy
import time
import serial
import math
import tf
from multiprocessing import Process, Value
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import Quaternion
from numpy import *
from tf.transformations import euler_from_quaternion

LEFT = 0
RIGHT = 1

PALUSE_NUM_PER_ROTATION = 8 # 一回転あたりのパルス数
REDUCTION_RATIO = 28        # ギアの減速比
WHEEL_RADIUS = 0.32         # 車輪の直径
TREAD = 0.5616				# 左右車輪間距離

COV = 0.0005356910249999999
COV_MATRIX = [1e-3, 0.0, 0.0, 0.0, 0.0, 0.0, 
	       	  0.0, 1e-3, 0.0, 0.0, 0.0, 0.0, 
	          0.0, 0.0, 1e6, 0.0, 0.0, 0.0, 
	          0.0, 0.0, 0.0, 1e-6, 0.0, 0.0, 
	          0.0, 0.0, 0.0, 0.0, 1e-6, 0.0, 
	          0.0, 0.0, 0.0, 0.0, 0.0, 1e3]

GEAR_CONSTANT = math.pi * WHEEL_RADIUS / (REDUCTION_RATIO * PALUSE_NUM_PER_ROTATION) #パルス→移動量計算用

def conecct_with_arduino_process(left_count,right_count):

	port = rospy.get_param('encorder_port',"/dev/ttyACM0")
	try:
		ser = serial.Serial(port,19200)
		ser.setDTR(False)
		time.sleep(1)
		ser.setDTR(True)
		print "start connection with rotarty encoder"
	except:
		print "cannot start connection with rotarty encoder"

	time.sleep(2)

	while  not rospy.is_shutdown():
		line  = ser.readline().rstrip()
		LorR  = line[:1]
		value = int(line[1:])
		#print "%s,%d" % (LorR,value)
		if LorR == "L":
			left_count.value = value
		elif LorR == "R":
			right_count.value = value

	ser.close()


def calculate_odometory_process(left_count,right_count):

	rospy.init_node('calculate_odometory_from_rotary_encoder', anonymous=True)
	pub = rospy.Publisher('encoded_odom', Odometry , queue_size=10)
	odom_broadcaster = tf.TransformBroadcaster()

	odo = Odometry()
	odom_trans = TransformStamped()

	current_time = rospy.get_rostime()
	last_time = rospy.get_rostime()
	current_count = [0,0]
	last_count = [0,0]

	odo.pose.pose.position.x = 0
	q = tf.transformations.quaternion_from_euler(0, 0, 0)
	msg = Quaternion(*q)
	odo.pose.pose.orientation = msg
	odo.twist.twist.linear.x = 0
 	odo.twist.twist.angular.z = 0

 	rate = rospy.Rate(10) # 10hz
    
	global dt
	global count

	count = 0
	dt = 0.1
	t = 0;

	while not rospy.is_shutdown():
		
		count = count + 1
		current_time = rospy.get_rostime()
		dt = current_time.to_sec() - last_time.to_sec()
		current_count[LEFT]  = left_count.value
		current_count[RIGHT] = right_count.value

		L = palse_to_deltaL_and_deltaTheta(current_count,last_count)
		odo = update_odometory(L[0], L[1], odo ,dt)
		odo.header.stamp = current_time
		odo.header.frame_id = "odom"
		odo.child_frame_id = "base_link"
		odo.pose.covariance  = COV_MATRIX
		odo.twist.covariance = COV_MATRIX

		pub.publish(odo)
		#rospy.loginfo("calc_p L*%d,R:%d",left_count.value,right_count.value)
		#rospy.loginfo("odom")

		# tf
		odom_trans.header.stamp = current_time
		odom_trans.header.frame_id = "odom"
		odom_trans.child_frame_id = "base_link"

		odom_trans.transform.translation.x = odo.pose.pose.position.x
		odom_trans.transform.translation.y = odo.pose.pose.position.y
		odom_trans.transform.translation.z = odo.pose.pose.position.z
		odom_trans.transform.rotation = odo.pose.pose.orientation

		#odom_broadcaster.sendTransform((odom_trans.transform.translation.x,odom_trans.transform.translation.y,0),
		#	(odom_trans.transform.rotation.x,odom_trans.transform.rotation.y,odom_trans.transform.rotation.z,odom_trans.transform.rotation.w),
		#	odom_trans.header.stamp,odom_trans.child_frame_id,odom_trans.header.frame_id)
		#################################
		
		last_time = rospy.get_rostime()
		last_count[RIGHT] = current_count[RIGHT]
		last_count[LEFT]  = current_count[LEFT]
		t = t + dt

		rate.sleep()

# この部分のアルゴリズムについては以下のurlを参照した
# http://www.mech.tohoku-gakuin.ac.jp/rde/contents/course/robotics/wheelrobot.html
def palse_to_deltaL_and_deltaTheta(now_c,last_c):
    u=array([0,0]);
    Lr = ( now_c[RIGHT] - last_c[RIGHT] ) * GEAR_CONSTANT
    Ll = ( now_c[LEFT]  - last_c[LEFT]  ) * GEAR_CONSTANT
    deltaL = ( Lr + Ll ) / 2.0
    deltaTheta = ( Lr - Ll ) / TREAD
    u = array([deltaL,deltaTheta])
    return u

def update_odometory(deltaL,deltaTheta,last_odom,dt):
	new_odom = Odometry()
	(roll,pitch,yaw) = euler_from_quaternion([last_odom.pose.pose.orientation.x,last_odom.pose.pose.orientation.y,last_odom.pose.pose.orientation.z,last_odom.pose.pose.orientation.w])

	#print yaw
	q = tf.transformations.quaternion_from_euler(0, 0, yaw + deltaTheta)
	msg = Quaternion(*q)
	new_odom.pose.pose.orientation = msg

	if abs(deltaTheta) < 0.05:
		new_odom.pose.pose.position.x = last_odom.pose.pose.position.x + deltaL * cos(yaw + deltaTheta/2.0)
		new_odom.pose.pose.position.y = last_odom.pose.pose.position.y + deltaL * sin(yaw + deltaTheta/2.0)
	else:
		deltaL_dash = 2.0 * (deltaL/deltaTheta) * sin(deltaTheta/2.0)
		new_odom.pose.pose.position.x = last_odom.pose.pose.position.x + deltaL_dash * cos(yaw + deltaTheta/2.0)
		new_odom.pose.pose.position.y = last_odom.pose.pose.position.y + deltaL_dash * sin(yaw + deltaTheta/2.0)
	new_odom.twist.twist.linear.x  = (deltaL / dt)*cos(yaw)
	new_odom.twist.twist.linear.y  = (deltaL / dt)*sin(yaw) 
	new_odom.twist.twist.angular.z = deltaTheta / dt	
	return new_odom

if __name__ == '__main__':
	right_count = Value('d',0)
	left_count  = Value('d',0)
	connect_process = Process( target = conecct_with_arduino_process , args = (left_count,right_count))
	calculate_process = Process( target = calculate_odometory_process, args = (left_count,right_count))
	try:
		connect_process.start()
		calculate_process.start()
	except rospy.ROSInterruptException:
		pass