#include "global.h"

#include <sstream>
#include <cstdlib>
#include <iostream>


std::vector< std::map< std::tuple<int, int>, Intersection > > _database;
map<string, Car> _car_dict;

shared_mutex _database_g_mutex;
unique_lock<shared_mutex> _database_wLock(_database_g_mutex);
shared_lock<shared_mutex> _database_rLock(_database_g_mutex);


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
	_database_wLock.lock();
	_database.push_back(intersection_map);
	_database_wLock.unlock();
}

void move_a_time_step() {
	_database_wLock.lock();
	if (_database.size() <= _routing_period_num) {
		_database.clear();
	}
	else {
		_database.erase(_database.begin(), _database.begin()+_routing_period_num);
	}
	_database_wLock.unlock();
}

Intersection& get_intersection(const int current_arrival_time, const tuple<int, int> &intersection_id) {
	while (current_arrival_time >= _database.size()) {
		add_time_step();
	}

	return _database[current_arrival_time][intersection_id];
}



// Update car info from SUMO (Called only by main thread)
void update_car(const string& car_id, const uint8_t& car_length, const string& src_intersection_id,
	const uint8_t& direction_of_src_intersection, const uint16_t& time_offset_step,
	const double& position_at_offset, const string& dst_node_id) {

	map<string, Car>::iterator iter = _car_dict.find(car_id);

	if (iter == _car_dict.end()) {
		stringstream ss(dst_node_id);
		string dst_id_substr = "";
		getline(ss, dst_id_substr, '_');
		int dst_id_1 = stoi(dst_id_substr);
		getline(ss, dst_id_substr, '_');
		int dst_id_2 = stoi(dst_id_substr);
		tuple<int, int> dst_coord(dst_id_1, dst_id_2);
		
		_car_dict[car_id] = Car(car_id, car_length, dst_coord);
	}

	stringstream ss(src_intersection_id);
	string src_id_substr = "";
	getline(ss, src_id_substr, '_');
	int src_id_1 = stoi(src_id_substr);
	getline(ss, src_id_substr, '_');
	int src_id_2 = stoi(src_id_substr);
	tuple<int, int> src_coord(src_id_1, src_id_2);

	_car_dict[car_id].src_coord = src_coord;
	_car_dict[car_id].direction_of_src_intersection = direction_of_src_intersection;
	_car_dict[car_id].time_offset_step = time_offset_step;
	_car_dict[car_id].position_at_offset = position_at_offset;
}

// Choose car for reroute (Called only by main thread)
vector<vector<reference_wrapper<Car>>> choose_car_to_thread_group(vector<string>& new_car_ids, vector<string>& old_car_ids) {
	vector<vector<reference_wrapper<Car>>> results;

	// Dummy allocation  TODO: change
	for (int i = 0; i < _thread_num; i++) {
		results.push_back(vector<reference_wrapper<Car>>());
	}

	int process_idx = 0;
	for (string car_id : new_car_ids) {
		process_idx++;
		process_idx %= _thread_num;
		results[process_idx].push_back(_car_dict[car_id]);
	}

	for (vector<reference_wrapper<Car>> car_group : results) {
		for (Car &car: car_group) {
			//delete_car_from_database(results[i][j]);
		}
	}

	return results;
}

void delete_car_from_database(Car &car) {

	for ( tuple<string, Intersection> &record : car.records_intersection_in_database) {
		string &type = get<0>(record);
		Intersection &intersection = get<1>(record);

		intersection.delete_car_from_database(car, type);
	}

	car.records_intersection_in_database.clear();
}