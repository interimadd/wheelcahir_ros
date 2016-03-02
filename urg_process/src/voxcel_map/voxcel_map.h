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
			VoxcelMapの中心座標を移動させる
		*/
		void MoveVoxcelMapCenter(float pos_x,float pos_y,float pos_z);


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

};


#endif