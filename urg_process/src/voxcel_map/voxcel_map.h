#ifndef __VOXCEL_MAP_H_INCLUDED__
#define __VOXCEL_MAP_H_INCLUDED__

#include <stdint.h>
#include <vector>
#include <stdio.h>
#include "sensor_msgs/PointCloud.h"
#include "geometry_msgs/Point.h"

using namespace std;

enum VoxelStatus{
  FREE = 0,
  UNKNOWN = 1,
  MARKED = 2,
};

class Voxcel_Map{

	public:

		float center_x;
		float center_y;
		float center_z;

		float last_calc_x;
		float last_calc_y;
		float last_calc_z;

		Voxcel_Map(float pos_x,float pos_y,int map_size_x,int map_size_y,int map_size_z,float horizontal_resolution,float vertical_resolution);

		/*
			LRFの点群情報をVoxcelに格納する
		*/
		void RecordSensorData(sensor_msgs::PointCloud laser_point_data);

		/*
			Voxcelの中で埋まってる点をPointCloudに代入する
		*/
		void VoxcelToPointCloud(sensor_msgs::PointCloud *out);

		/*
			VoxcelMapの中心座標を移動させる,
		*/
		void MoveVoxcelMapCenter(float pos_x,float pos_y,float pos_z);

		/*
			タイヤの大きさを指定して、転動の可否を判断するパラメータを計算
		*/
		void SetTireRadius(float tire_radius,float tire_width);


	private:

		int MAP_SIZE_X;
		int MAP_SIZE_Y;
		int MAP_SIZE_Z;
		float HORIZONTAL_RESOLUTION;
		float VERTICAL_RESOLUTION;
		vector < vector <vector<int8_t> > > voxcel_map;

		/*
			配列の番号から実世界座標に変換するのに使う
		*/
		geometry_msgs::Point32 TranslateIndexToRealCordinate(int x_index,int y_index,int z_index);

		/*
			あるグリッド上でx方向,y方向にそれぞれ転動可能かを識別する
			tire_height:車輪中心を(0,0)としたとき、車輪の外周はグリッドの中で何マス目にあるのかを計算しておく
			enable_rolling_threshold:車輪のボクセルとの接触角度が60°以上で転動できないため、60°がグリッドになったタイヤのどこの座標に相等するかを計算しておく
		*/
		bool isEnableRolling(int x_index,int y_index);
		vector<int> tire_height;
		int half_tire_width;
		int enable_rolling_threshold;

		int returnMaxHeightZindex(int x_index,int y_index,int min_z_index,int max_z_index);

};


#endif