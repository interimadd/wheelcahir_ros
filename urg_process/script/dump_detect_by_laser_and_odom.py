#!/usr/bin/env python  
# coding: UTF-8

##tfの時間に関するエラー→callback毎にlistnerを生成していたため、バッファが持てていなかった？？

import rospy
import tf
from copy import deepcopy

from sensor_msgs.msg import PointCloud
from geometry_msgs.msg import Vector3
from numpy import *
from urg_process.msg import VectorField

#(7.2,400)(0.5,50)
UPDATE_DISTANCE = 0.5               # 地図情報更新処理を行う距離
HEIGHT_MAP_DATA_GRID_LENGTH = 0.025  # グリッドの一辺の長さ
HEIGHT_MAP_DATA_NUM = 50            # 100なら200*200のマップ
NOT_UPDATE = -1                      # 未更新のグリッドの高さ
SHOW_RATE = 2                        # 勾配矢印を表示する数を何分の一にするか

class HeightGridMap:

    center_x = 0
    center_y = 0

    last_calc_x = 0
    last_calc_y = 0
    
    def __init__(self,x,y):
        self.map =  [[ [NOT_UPDATE,0] for col in range(2*HEIGHT_MAP_DATA_NUM)] for row in range(2*HEIGHT_MAP_DATA_NUM)] # mapは[高さ,更新数]で定義
        self.gradient_map = [[ [0,0,0] for col in range(2*HEIGHT_MAP_DATA_NUM)] for row in range(2*HEIGHT_MAP_DATA_NUM)] # gradient_mapは[Vx,Vy,更新数]で定義 
        self.center_x = x
        self.center_y = y
        self.last_calc_x = x
        self.last_calc_y = y
        self.pub = rospy.Publisher('dump_gradient', VectorField, queue_size=10)
        self.pub_p = rospy.Publisher('height_point', VectorField, queue_size=10)

    def update(self,update_pos):
        #for i in range(1,HEIGHT_MAP_DATA_NUM*2-1):
        #    for j in range(1,HEIGHT_MAP_DATA_NUM*2-1):
        #        if self.map[i][j][1] > 5:
        #            rospy.loginfo("i %d,j %d,h %f,n %f",i,j,self.map[i][j][0],self.map[i][j][1])
        self.last_calc_x = update_pos[0]
        self.last_calc_y = update_pos[1]
        self.interpolate_map()
        self.calculate_map_gradient()
        self.publish_gradient()
        self.publish_height_data()
        self.move_map(update_pos)

    def move_map(self,move_pos):
        # 高さマップ、勾配マップの座標を移動させる
        tmp_map = [[ [NOT_UPDATE,0] for col in range(2*HEIGHT_MAP_DATA_NUM)] for row in range(2*HEIGHT_MAP_DATA_NUM)]
        tmp_grad_map = [[ [0,0,0] for col in range(2*HEIGHT_MAP_DATA_NUM)] for row in range(2*HEIGHT_MAP_DATA_NUM)] 
        for i in range(1,HEIGHT_MAP_DATA_NUM*2-1):
            for j in range(1,HEIGHT_MAP_DATA_NUM*2-1):
                if self.map[i][j][1] > 0:
                    n = (i - HEIGHT_MAP_DATA_NUM)*HEIGHT_MAP_DATA_GRID_LENGTH + self.center_x # 絶対座標系に戻す
                    m = (j - HEIGHT_MAP_DATA_NUM)*HEIGHT_MAP_DATA_GRID_LENGTH + self.center_y
                    new_n = int(HEIGHT_MAP_DATA_NUM + ( n - move_pos[0] ) / HEIGHT_MAP_DATA_GRID_LENGTH) # 新しい座標系に変換する
                    new_m = int(HEIGHT_MAP_DATA_NUM + ( m - move_pos[1] ) / HEIGHT_MAP_DATA_GRID_LENGTH)
                    if 0 < new_n and new_n < HEIGHT_MAP_DATA_NUM*2 and 0 < new_m and new_m < HEIGHT_MAP_DATA_NUM*2:
                        tmp_map[new_n][new_m][0] = self.map[i][j][0]
                        tmp_map[new_n][new_m][1] = self.map[i][j][1]
                        tmp_grad_map[new_n][new_m][0] = self.gradient_map[i][j][0]
                        tmp_grad_map[new_n][new_m][1] = self.gradient_map[i][j][1] 
                        tmp_grad_map[new_n][new_m][2] = self.gradient_map[i][j][2] 
        self.map = tmp_map
        self.gradient_map = tmp_grad_map

        self.center_x = move_pos[0]
        self.center_y = move_pos[1]
        return

    # 各グリッドの勾配を計算
    def calculate_map_gradient(self):
        CALC_CONST = HEIGHT_MAP_DATA_GRID_LENGTH * 2.0 * 3.0 # 2マス分の変位かつ3マス分のカウントなので
        for i in range(2,HEIGHT_MAP_DATA_NUM*2-2):
            for j in range(2,HEIGHT_MAP_DATA_NUM*2-2):
                if self.map[i][j][1] > 0:   # 更新済みのマスであれば計算
                    self.gradient_map[i][j][0] = (self.map[i+1][j+1][0] + self.map[i+1][j][0] + self.map[i+1][j-1][0] - self.map[i-1][j+1][0] - self.map[i-1][j][0] - self.map[i-1][j-1][0]) / CALC_CONST
                    self.gradient_map[i][j][1] = (self.map[i+1][j+1][0] + self.map[i][j+1][0] + self.map[i-1][j+1][0] - self.map[i+1][j-1][0] - self.map[i][j-1][0] - self.map[i-1][j-1][0]) / CALC_CONST
                    self.gradient_map[i][j][2] += 1

    # マップ情報をセンサモデルをもとに、検出できていないところを予測して更新
    def interpolate_map(self):
        #グリッドの抜けが多いので、高さ情報が記録されていなければ周りのマスを参照して、中央値を自分の高さとするアルゴリズムで補間
        tmp_map = deepcopy(self.map)
        near_grid = [[-1,-1],[-1,0],[-1,1],[0,-1],[0,1],[1,-1],[1,0],[1,-1]]
        for i in range(1,HEIGHT_MAP_DATA_NUM*2-1):
            for j in range(1,HEIGHT_MAP_DATA_NUM*2-1):
                if self.map[i][j][1] == 0:
                    tmp_grid = []
                    for n in range(0,len(near_grid)):
                        if tmp_map[ i + near_grid[n][0] ][ j + near_grid[n][1] ][1] > 0:
                            tmp_grid.append( tmp_map[ i + near_grid[n][0] ][ j + near_grid[n][1] ][0] )
                    if len(tmp_grid)>0:
                        tmp_grid.sort()
                        self.map[i][j][0] = tmp_grid[int(len(tmp_grid)/2)]
                        self.map[i][j][1] = 1

        #大きい穴を埋めるための補間 より遠くの周囲のマスを参照して、一番低いところの高さに合わせる
        tmp_map = deepcopy(self.map)
        #near_grid = [[-4,0],[0,-4],[0,4],[4,0]]
        near_grid = [[-2,-2],[-2,0],[-2,2],[0,-2],[0,2],[2,-2],[2,0],[2,-2]]
        for i in range(5,HEIGHT_MAP_DATA_NUM*2-5):
            for j in range(5,HEIGHT_MAP_DATA_NUM*2-5):
                if self.map[i][j][1] == 0:
                    tmp_grid = []
                    for n in range(0,len(near_grid)):
                        if tmp_map[ i + near_grid[n][0] ][ j + near_grid[n][1] ][1] > 0:
                            tmp_grid.append( tmp_map[ i + near_grid[n][0] ][ j + near_grid[n][1] ][0] )
                    if len(tmp_grid)>1:
                        tmp_grid.sort()
                        self.map[i][j][0] = tmp_grid[int(len(tmp_grid)/2)]
                        self.map[i][j][1] = 1

        return 0


    # レーザー絶対座標情報をマップに記録
    def record_sensor_data(self,height_data):
        for i in range(0,len(height_data)):
            # 絶対座標をグリッドの番号に変換
            num_x = int(HEIGHT_MAP_DATA_NUM + ( height_data[i][0] - self.center_x ) / HEIGHT_MAP_DATA_GRID_LENGTH)
            num_y = int(HEIGHT_MAP_DATA_NUM + ( height_data[i][1] - self.center_y ) / HEIGHT_MAP_DATA_GRID_LENGTH)
            if abs(num_x) < HEIGHT_MAP_DATA_NUM*2 and abs(num_y) < HEIGHT_MAP_DATA_NUM*2:
                #self.map[num_x][num_y][0] = (self.map[num_x][num_y][0]*self.map[num_x][num_y][1] + height_data[i][2]) / (self.map[num_x][num_y][1] + 1) # これまで得られている値との平均を記録
                #self.map[num_x][num_y][0] = height_data[i][2]
                if self.map[num_x][num_y][0] < height_data[i][2]:
                    self.map[num_x][num_y][0] = height_data[i][2]
                self.map[num_x][num_y][1] += 1
                #rospy.loginfo("%f,%f,%f,%d,%d,%f,%d",height_data[i][0],height_data[i][1],height_data[i][2],num_x,num_y,self.map[num_x][num_y][0],self.map[num_x][num_y][1])

    def publish_gradient(self):
        grad_data = VectorField()
        grad_data.child_frame_id = "odom"
        count = 0
        for i in range(1,HEIGHT_MAP_DATA_NUM*2-1):
            for j in range(1,HEIGHT_MAP_DATA_NUM*2-1):
                absgrad = self.gradient_map[i][j][0]*self.gradient_map[i][j][0] + self.gradient_map[i][j][1]*self.gradient_map[i][j][1]
                if (0.03*0.03)<absgrad and absgrad<1000000:
                    if count % SHOW_RATE == 0:
                        tmp_pos = Vector3()
                        tmp_pos.x = (i - HEIGHT_MAP_DATA_NUM)*HEIGHT_MAP_DATA_GRID_LENGTH + self.center_x
                        tmp_pos.y = (j - HEIGHT_MAP_DATA_NUM)*HEIGHT_MAP_DATA_GRID_LENGTH + self.center_y
                        tmp_pos.z = 0
                        tmp_vec = Vector3()
                        tmp_vec.x = self.gradient_map[i][j][0]
                        tmp_vec.y = self.gradient_map[i][j][1]
                        tmp_vec.z = 0
                        grad_data.pos.append(tmp_pos)
                        grad_data.vec.append(tmp_vec)
                    count += 1
        grad_data.data_num = len(grad_data.pos)
        self.pub.publish(grad_data)

    def publish_height_data(self):
        height_data = VectorField()
        height_data.child_frame_id = "odom"
        for i in range(1,HEIGHT_MAP_DATA_NUM*2-1):
            for j in range(1,HEIGHT_MAP_DATA_NUM*2-1):
                if self.map[i][j][1] > 0:
                    tmp_pos = Vector3()
                    tmp_pos.x = (i - HEIGHT_MAP_DATA_NUM)*HEIGHT_MAP_DATA_GRID_LENGTH + self.center_x
                    tmp_pos.y = (j - HEIGHT_MAP_DATA_NUM)*HEIGHT_MAP_DATA_GRID_LENGTH + self.center_y
                    tmp_pos.z = self.map[i][j][0]
                    height_data.pos.append(tmp_pos)
        height_data.data_num = len(height_data.pos)
        self.pub_p.publish(height_data)




class ScanDataProcesser:
    def __init__(self):
        rospy.init_node('bump_detect')
        self.grid_map_data = HeightGridMap(0,0)
        self.listener = tf.TransformListener()

    def subscribe_scan_data(self):
        rospy.Subscriber("laser_point", PointCloud,self.callback)
        rospy.spin()

    def callback(self,data):
        # laser系の座標をグローバル変数に格納
        
        now = rospy.Time.now()
        self.listener.waitForTransform("/map_center", "/odom", now, rospy.Duration(1.0))
        (trans,rot) = self.listener.lookupTransform("odom","map_center", now)

        sensor_data_height = []
        for i in range(0,len(data.points)):
            if abs(data.points[i].z) < 0.6: 
                sensor_data_height.append( [ data.points[i].x , data.points[i].y , data.points[i].z ] )

        self.grid_map_data.record_sensor_data(sensor_data_height)  # 絶対座標系レーザーデータをグリッド高さマップに格納

        # 一定距離移動したらマップデータの勾配計算処理を行う
        if (self.grid_map_data.last_calc_x - trans[0]) * (self.grid_map_data.last_calc_x - trans[0]) + \
            (self.grid_map_data.last_calc_y - trans[1]) * (self.grid_map_data.last_calc_y - trans[1]) > UPDATE_DISTANCE * UPDATE_DISTANCE:
            self.grid_map_data.update(trans)
            #rospy.loginfo("lc_x %f,lf_x %f,lc_y %f,lf_y %f",self.grid_map_data.last_calc_x,self.laser_frame_trans[0],self.grid_map_data.last_calc_y,self.laser_frame_trans[1])
        

if __name__ == '__main__':
    processer = ScanDataProcesser()
    processer.subscribe_scan_data()
