#!/usr/bin/env python  
# coding: UTF-8

#オドメトリトピックをtfに変換

import rospy
import tf

from nav_msgs.msg import Odometry
from numpy import *
from tf.transformations import euler_from_quaternion

x = 0
y = 0
th = 0

def callback(last_odom):
    global x
    global y
    global th
    x = last_odom.pose.pose.position.x
    y = last_odom.pose.pose.position.y
    (roll,pitch,yaw) = euler_from_quaternion([last_odom.pose.pose.orientation.x,last_odom.pose.pose.orientation.y,last_odom.pose.pose.orientation.z,last_odom.pose.pose.orientation.w])
    th = yaw


if __name__ == '__main__':

    topic = rospy.get_param('odom_topic',"seniorcar_odometry")
    offset = rospy.get_param('offset_between_ground_and_base_link',0)
    print topic
    rospy.init_node('laser_to_odom_broadcaster')
    rospy.Subscriber(topic, Odometry, callback)

    br = tf.TransformBroadcaster()
    rate = rospy.Rate(10.0)

    while not rospy.is_shutdown():
        q = tf.transformations.quaternion_from_euler(0, 0, th)
        br.sendTransform((x, y, offset),q,rospy.Time.now(),"base_link","odom")
        #print x,y,th
        rate.sleep()
