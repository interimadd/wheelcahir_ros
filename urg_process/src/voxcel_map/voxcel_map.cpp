#include "voxcel_map.h"

Voxcel_Map::Voxcel_Map(float pos_x,float pos_y,int map_size_x,int map_size_y,int map_size_z,float horizontal_resolution,float vertical_resolution){

	MAP_SIZE_X = map_size_x;
	MAP_SIZE_Y = map_size_y;
	MAP_SIZE_Z = map_size_z;

	center_x = pos_x;
    center_y = pos_y;

    last_calc_x = pos_x;
    last_calc_y = pos_y;

	HORIZONTAL_RESOLUTION = horizontal_resolution;
	VERTICAL_RESOLUTION = vertical_resolution;

	voxcel_map.resize(map_size_x*2);

	for(int i=0; i < map_size_x*2 ;i++){
		voxcel_map[i].resize(map_size_y*2);
		for(int j=0; j < map_size_y*2 ;j++){
			voxcel_map[i][j].resize(map_size_z*2);
		}
	}

}


void Voxcel_Map::RecordSensorData(sensor_msgs::PointCloud laser_point_data){
	
	int num_x,num_y,num_z;
	int DATA_NUM = laser_point_data.points.size();

	for(int i=0;i<DATA_NUM;i++){
		num_x = int( float(MAP_SIZE_X) + ( laser_point_data.points[i].x - center_x ) / HORIZONTAL_RESOLUTION );
		if( 0 <= num_x && num_x < MAP_SIZE_X * 2 ){
			num_y = int( float(MAP_SIZE_Y) + ( laser_point_data.points[i].y - center_y ) / HORIZONTAL_RESOLUTION );
			if( 0 <= num_y && num_y < MAP_SIZE_Y * 2 ){
				num_z = int( float(MAP_SIZE_Z) + ( laser_point_data.points[i].z - center_z ) / VERTICAL_RESOLUTION );
				if( 0 <= num_z && num_z < MAP_SIZE_Z * 2){
					voxcel_map[num_x][num_y][num_z] = MARKED;
				}
			}
		}
	}

	return;
}


void Voxcel_Map::VoxcelToPointCloud(sensor_msgs::PointCloud *out){

	for(int i=0 ; i < MAP_SIZE_X * 2 ; i++){
		for(int j=0 ; j < MAP_SIZE_Y * 2 ;j++){
			for(int k=0 ; k < MAP_SIZE_Z * 2 ;k++){
				if( voxcel_map[i][j][k] == MARKED){
					out->points.push_back(TranslateIndexToRealCordinate(i,j,k));
				}
			}
		}
	}

}


inline geometry_msgs::Point32 Voxcel_Map::TranslateIndexToRealCordinate(int x_index,int y_index,int z_index){
	geometry_msgs::Point32 point;
	point.x = float(x_index - MAP_SIZE_X) * HORIZONTAL_RESOLUTION + center_x;
	point.y = float(y_index - MAP_SIZE_Y) * HORIZONTAL_RESOLUTION + center_y;
	point.z = float(z_index - MAP_SIZE_Z) *   VERTICAL_RESOLUTION + center_z;
	return point;
}


// 効率の良いアルゴリズムを考えたい
void Voxcel_Map::MoveVoxcelMapCenter(float pos_x,float pos_y,float pos_z){

	// 複製用のvector
	vector < vector <vector<int8_t> > > tmp_map;
	tmp_map.resize( MAP_SIZE_X * 2);
	for(int i=0; i < MAP_SIZE_X * 2 ;i++){
		tmp_map[i].resize(MAP_SIZE_Y * 2 );
		for(int j=0; j < MAP_SIZE_Y * 2 ;j++){
			tmp_map[i][j].resize( MAP_SIZE_Z * 2 );
		}
	}

	// vectorのコピーともとのvectorの初期化
	for(int i=0 ; i < MAP_SIZE_X * 2 ; i++){
		for(int j=0 ; j < MAP_SIZE_Y * 2 ;j++){
			for(int k=0 ; k < MAP_SIZE_Z * 2 ;k++){
				tmp_map[i][j][k] = voxcel_map[i][j][k];
				voxcel_map[i][j][k] = FREE;
			}
		}
	}
	int count = 0;
	int count2 = 0;
	// 複製vectorを移動させる
	int new_x_index,new_y_index,new_z_index;
	for(int i=0 ; i < MAP_SIZE_X * 2 ; i++){
		for(int j=0 ; j < MAP_SIZE_Y * 2 ;j++){
			for(int k=0 ; k < MAP_SIZE_Z * 2 ;k++){
				if(tmp_map[i][j][k] == MARKED){
					count2++;
					new_x_index = i + (center_x - pos_x) / HORIZONTAL_RESOLUTION;
					if( 0 <= new_x_index && new_x_index < MAP_SIZE_X * 2 ){
						new_y_index = j + (center_y - pos_y) / HORIZONTAL_RESOLUTION;
						if( 0 <= new_y_index && new_y_index < MAP_SIZE_Y * 2 ){
							new_z_index = k + (center_z - pos_z) / VERTICAL_RESOLUTION;
							if( 0 <= new_z_index && new_z_index < MAP_SIZE_Z * 2 ){
								voxcel_map[new_x_index][new_y_index][new_z_index] = MARKED;
								count++;
							}
						}
					}
				}
			}
		}
	}

	center_x = pos_x;
	center_y = pos_y;
	center_z = pos_z;

	printf("count:%d,%d\n",count2,count);

}


inline bool Voxcel_Map::isEnableRolling(int x_index,int y_index){

	int tmp_max_x = 0;
	int tmp_map_y = 0;
	int tmp_max_z = 0;

	//for(int i=0; i < tire_radius; i++){
		
	//}
	return true;
}


void Voxcel_Map::SetTireRadius(float tire_radius,float tire_width){

	enable_rolling_threshold = tire_radius / (2.0 * HORIZONTAL_RESOLUTION);
	half_tire_width = tire_width / (2.0 * HORIZONTAL_RESOLUTION);

	int max_i = tire_radius / HORIZONTAL_RESOLUTION;
	for(int i=0;i<max_i;i++){
		int tmp_height = (sqrt(pow(tire_radius,2) - pow(HORIZONTAL_RESOLUTION*i,2))) / VERTICAL_RESOLUTION;
		tire_height.push_back(tmp_height);
	}

}


inline int Voxcel_Map::returnMaxHeightZindex(int x_index,int y_index,int min_z_index,int max_z_index){

	int max_i = max_z_index - min_z_index;
	for(int i=0;i<max_i;i++){
		if( voxcel_map[x_index][y_index][max_z_index-i] == MARKED){
			return max_z_index - i;
		}
	}

	return min_z_index;

}