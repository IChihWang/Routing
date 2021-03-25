#include "global.h"

std::vector< std::map< std::tuple<int, int>, Intersection > > _database;


/* Handling database */

void create_grid_network() {
	for (int idx = 0; idx < DEFAULT_DATA_LENGTH; idx++) {
		add_time_step();
	}
}

void add_time_step() {
	std::map< std::tuple<int, int>, Intersection > intersection_map;
	for (int i = 1; i <= _grid_size; i++) {
		for (int j = 1; j <= _grid_size; j++) {
			tuple<int, int> coordinate(i, j);
			Intersection intersection(coordinate);
			intersection_map[coordinate] = intersection;
		}
	}

	// TODO: connect intersections
	_database.push_back(intersection_map);
}

void move_a_time_step() {
	if (_database.size() <= _routing_period_num) {
		_database.clear();
	}
	else {
		_database.erase(_database.begin(), _database.begin()+_routing_period_num);
	}
}

Intersection get_intersection(int current_arrival_time, tuple<int, int> intersection_id) {
	while (current_arrival_time >= _database.size()) {
		add_time_step();
	}

	return _database[current_arrival_time][intersection_id];
}