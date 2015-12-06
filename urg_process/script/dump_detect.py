#!/usr/bin/env python
# coding: UTF-8

import rospy
import math
from sensor_msgs.msg import LaserScan

CHANGE = 1
FLAT = 0

LASER_HEIGHT = 1.2  #LRF設置位置の地面からの嵩さ
DETECT_THRETHOLD_HEIGHT_UPPER_X = 0.01
DETECT_THRETHOLD_HEIGHT_UPPER_Y = 0.01
DETECT_THRETHOLD_HEIGHT_DOWNER_X = 0.01
DETECT_THRETHOLD_HEIGHT_DOWNER_Y = 0.05
GRID_LENGTH = 0.02  #LRF情報をどの範囲でまとめるか

def callback(data):

    data_num = len(data.ranges)

    #LRF極座標データを直行座標データに変換(x,h)
    sensor_data_height = []
    for i in range(0,data_num-1):
        if data.ranges[i] < data.range_max and data.ranges[i] > data.range_min :
            sensor_data_height.append( [data.ranges[i] * math.sin( data.angle_min + data.angle_increment * i ), LASER_HEIGHT - data.ranges[i] * math.cos( data.angle_min + data.angle_increment * i )] )
    #sensor_data_height.sort()
    #print(sensor_data_height)

    """
    #LRF情報をグリッドの平均をとってまとめる
    sensor_data_height_grid = []
    grid_start = sensor_data_height[0][0]
    sum = 0
    sum_num = 0
    for i in range(0,len(sensor_data_height)-1):
        if grid_start + GRID_LENGTH > sensor_data_height[i][0]:
            sum += sensor_data_height[i][1]
            sum_num += 1
        else:
            if sum_num > 0:
                sensor_data_height_grid.append([ grid_start + GRID_LENGTH/2.0 , sum/sum_num])
            sum = sensor_data_height[i][1]
            sum_num = 1
            grid_start = sensor_data_height[i][0]
    """

    """
    for i in range(0,len(sensor_data_height_grid)):
        rospy.loginfo("x:%f,h:%f",sensor_data_height_grid[i][0],sensor_data_height_grid[i][1])
    """

    step_list = []
    detect_step_flag = FLAT
    step_start = 0
    #起伏から平坦を段差の始点、平坦から起伏の場所を段差の終点として記録
    for i in range(1,len(sensor_data_height)-1):
        if abs( sensor_data_height[i][1] - sensor_data_height[i-1][1] ) > DETECT_THRETHOLD_HEIGHT_UPPER_Y and abs(sensor_data_height[i][0] - sensor_data_height[i-1][0]) < DETECT_THRETHOLD_HEIGHT_UPPER_X:
            if detect_step_flag == FLAT:
                step_list.append([step_start,i])
            detect_step_flag = CHANGE
        elif sensor_data_height[i-1][1] - sensor_data_height[i][1]  > DETECT_THRETHOLD_HEIGHT_DOWNER_Y and abs(sensor_data_height[i][0] - sensor_data_height[i-1][0]) > DETECT_THRETHOLD_HEIGHT_DOWNER_X:
            step_list.append([step_start,i-1])
            step_start = i
            detect_step_flag = FLAT
        else:
            if detect_step_flag == CHANGE:
                step_start = i
            detect_step_flag = FLAT

    step_list.append([step_start,i])

    for i in range(0,len(step_list)):
        s_s = step_list[i][0]
        s_e = step_list[i][1]
        h_sum = 0
        for n in range(s_s,s_e):
            h_sum += sensor_data_height[n][1]
        h_sum = h_sum / (s_e - s_s)
        if  sensor_data_height[s_e][0] - sensor_data_height[s_s][0] > 0.05:
            rospy.loginfo("distance:%f,height:%f,length:%f",sensor_data_height[s_s][0], h_sum , sensor_data_height[s_e][0] - sensor_data_height[s_s][0])
        #rospy.loginfo("s_s:%d,e_s:%d",s_s,s_e)

    rospy.loginfo("loop")


"""
    for i in range(1,data_num-1):
        h = data.ranges[i] * math.cos( data.angle_min + data.angle_increment * i )
        delta_h = h - pref_h
        pref_h = h
        if abs(delta_h) > DETECT_THRETHOLD_HEIGHT:
            rospy.loginfo("delta_h:%f,distance%f",delta_h,data.ranges[i]*math.sin(data.angle_min + data.angle_increment*i))
"""

def listener():

    rospy.init_node('dump_detect', anonymous=True)

    rospy.Subscriber("scan", LaserScan, callback)

    rospy.spin()

if __name__ == '__main__':
    listener()