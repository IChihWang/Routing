#include "global.h"
#include <cmath>
using namespace std;

map< Coord, Coord> _intersection_MEC;

void initial_district_allocation() {
	int _num_intersection_per_edge = ceil(1.0 * _grid_size / _MEC_num_per_edge);

	for (int i = 1; i <= _grid_size; i++) {
		for (int j = 1; j <= _grid_size; j++) {
			_intersection_MEC[Coord(i, j)] = Coord((i-1) / _num_intersection_per_edge, (j-1) / _num_intersection_per_edge);
		}
	}
}