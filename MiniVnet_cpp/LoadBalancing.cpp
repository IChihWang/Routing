#include "global.h"
#include <cmath>
using namespace std;

map< Coord, Coord> _intersection_MEC;
map< string, Coord>	_car_id_MEC_map;

void initial_district_allocation() {
	int _num_intersection_per_edge = ceil(1.0 * _grid_size / _MEC_num_per_edge);

	for (int i = 1; i <= _grid_size; i++) {
		for (int j = 1; j <= _grid_size; j++) {
			Coord MEC_id = Coord((i - 1) / _num_intersection_per_edge, (j - 1) / _num_intersection_per_edge);
			_MEC_id_list.push_back(MEC_id);
			_intersection_MEC[Coord(i, j)] = MEC_id;
		}
	}
}

void put_cars_into_districts(){
	for (auto& [car_id, car] : _car_dict) {
		Coord& MEC_id = _intersection_MEC[car.src_coord];
		_car_id_MEC_map[car_id] = MEC_id;
	}
}
