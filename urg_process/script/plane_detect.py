#!/usr/bin/env python
# coding: UTF-8

import rospy
import math
from sensor_msgs.msg import LaserScan

CHANGE = 1
FLAT = 0

LASER_HEIGHT = 1.2  #LRF設置位置の地面からの嵩さ
DETECT_THRETHOLD_HEIGHT = 0.005
DETECT_THRETHOLD_DISTANCE = 0.03
GRID_LENGTH = 0.02  #LRF情報をどの範囲でまとめるか

def callback(data):

    data_num = len(data.ranges)

    #LRF極座標データを直行座標データに変換(x,h)
    sensor_data_height = []
    for i in range(0,data_num-1):
        if data.ranges[i] < data.range_max and data.ranges[i] > data.range_min :
            d = data.ranges[i] * math.sin( data.angle_min + data.angle_increment * i )
            h = LASER_HEIGHT - data.ranges[i] * math.cos( data.angle_min + data.angle_increment * i )
            if abs(d) < 0.1 :
                h = 0
            sensor_data_height.append( [ d , h ] )

    plane_list = []
    detect_plane_flag = FLAT
    plane_start = 0
    #起伏から平坦を平地の始点、平坦から起伏の場所を平地の終点として記録
    for i in range(1,len(sensor_data_height)-1):
        if abs( sensor_data_height[i][1] - sensor_data_height[i-1][1] ) < DETECT_THRETHOLD_HEIGHT and abs(sensor_data_height[i][0] - sensor_data_height[i-1][0]) < DETECT_THRETHOLD_DISTANCE:
            if detect_plane_flag == CHANGE:
                plane_start = i
            detect_plane_flag = FLAT
        else:
            if detect_plane_flag == FLAT:
                plane_list.append([plane_start,i,0])
            detect_plane_flag = CHANGE

    plane_list.append([plane_start,i,0])

    #平地区間の嵩さの平均値を求める
    for i in range(0,len(plane_list)):
        s_s = plane_list[i][0]
        s_e = plane_list[i][1]
        h_sum = 0
        for n in range(s_s,s_e):
            h_sum += sensor_data_height[n][1]
        h_sum = h_sum / (s_e - s_s)
        plane_list[i][2] = h_sum
        #rospy.loginfo("distance:%f,height:%f,length:%f",sensor_data_height[s_s][0], h_sum , sensor_data_height[s_e][0] - sensor_data_height[s_s][0])

    #細切れになっている平地情報を、嵩さがあまり変わらなければつなげる
    plane_list_compressed = []
    plane_list_compressed.append([plane_list[0][0],plane_list[0][1],plane_list[0][2]])
    commpressed_index = 0
    for i in range(1,len(plane_list)):
        if abs( plane_list[i][2] - plane_list_compressed[commpressed_index][2] ) < 0.02:
            mean_height = ( ( sensor_data_height[plane_list[i][1]][0] - sensor_data_height[plane_list[i][0]][0] ) * plane_list[i][2] + \
            ( sensor_data_height[plane_list_compressed[commpressed_index][1]][0] - sensor_data_height[plane_list_compressed[commpressed_index][0]][0] ) * plane_list_compressed[commpressed_index][2] ) / \
            (sensor_data_height[plane_list[i][1]][0] - sensor_data_height[plane_list[i][0]][0] + sensor_data_height[plane_list_compressed[commpressed_index][1]][0] - sensor_data_height[plane_list_compressed[commpressed_index][0]][0] )
            plane_list_compressed[commpressed_index][1] = plane_list[i][1]
            plane_list_compressed[commpressed_index][2] = mean_height
        else:
            plane_list_compressed.append( [ plane_list[i][0] , plane_list[i][1] , plane_list[i][2] ])
            commpressed_index += 1

    rospy.loginfo("compress")

    for i in range(0,len(plane_list_compressed)):
        s_s = plane_list_compressed[i][0]
        s_e = plane_list_compressed[i][1]
        length = sensor_data_height[s_e][0] - sensor_data_height[s_s][0]
        if length > 0.02:
            rospy.loginfo("distance:%f,height:%f,length:%f",sensor_data_height[s_s][0], plane_list_compressed[i][2] , sensor_data_height[s_e][0] - sensor_data_height[s_s][0])

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

    rospy.init_node('plane_detect', anonymous=True)

    rospy.Subscriber("scan", LaserScan, callback)

    rospy.spin()

if __name__ == '__main__':
    listener()