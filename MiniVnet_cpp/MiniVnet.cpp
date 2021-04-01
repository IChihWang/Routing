#include "global.h"

#include <sstream>
#include <cstdlib>
#include <iostream>


std::vector< std::map< std::tuple<int, int>, Intersection > > _database;
map<string, Car> _car_dict;

shared_mutex _database_g_mutex;
//unique_lock<shared_mutex> _database_wLock(_database_g_mutex);
//shared_lock<shared_mutex> _database_rLock(_database_g_mutex);



/* Handling database */

void create_grid_network() {
	for (int idx = 0; idx < DEFAULT_DATA_LENGTH; idx++) {
		add_time_step();
	}
}

void add_time_step() {
	map< Coord, Intersection > intersection_map;
	for (int i = 1; i <= _grid_size; i++) {
		for (int j = 1; j <= _grid_size; j++) {
			Coord coordinate(i, j);
			Intersection intersection(coordinate);
			intersection_map[coordinate] = intersection;
		}
	}
	connect_intersections(intersection_map);

	_database.push_back(intersection_map);
}

void connect_intersections(map< Coord, Intersection >& intersection_map) {
	for (int i = 1; i <= _grid_size; i++) {
		for (int j = 1; j <= _grid_size; j++) {
			Coord coordinate(i, j);
			if (i <= _grid_size - 1) {
				Coord target_coordinate(i + 1, j);
				intersection_map[coordinate].connect(1, intersection_map[target_coordinate], 3);
			}
			if (j <= _grid_size - 1) {
				Coord target_coordinate(i, j + 1);
				intersection_map[coordinate].connect(2, intersection_map[target_coordinate], 0);
			}
		}
	}
}

void move_a_time_step() {
	
	if (int(_database.size()) <= _routing_period_num) {
		_database.clear();
	}
	else {
		cout << "    " << int(_database.size()) << "  a|  " << (int)_routing_period_num << endl;
		_database.erase(_database.begin(), _database.begin()+_routing_period_num);
		cout << "    " << int(_database.size()) << "  b|  " << (int)_routing_period_num << endl;
	}
}

Intersection& get_intersection(const int current_arrival_time, const Coord &intersection_id) {
	while (current_arrival_time >= int(_database.size())) {
		add_time_step();
	}
	_database[current_arrival_time];
	cout << _database[current_arrival_time][intersection_id].AZ_accumulated_size << endl;
	Intersection& intersection = _database[current_arrival_time][intersection_id];
	return intersection;
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
		Coord dst_coord(dst_id_1, dst_id_2);
		
		_car_dict[car_id] = Car(car_id, car_length, dst_coord);
	}

	stringstream ss(src_intersection_id);
	string src_id_substr = "";
	getline(ss, src_id_substr, '_');
	int src_id_1 = stoi(src_id_substr);
	getline(ss, src_id_substr, '_');
	int src_id_2 = stoi(src_id_substr);
	Coord src_coord(src_id_1, src_id_2);

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
			delete_car_from_database(car);
		}
	}

	return results;
}

void delete_car_from_database(Car &car) {

	for (tuple<string, reference_wrapper<Intersection>>&record : car.records_intersection_in_database) {
		string &type = get<0>(record);
		Intersection &intersection = get<1>(record);

		intersection.delete_car_from_intersection(car, type);
	}

	car.records_intersection_in_database.clear();
}

void delete_car_from_database_id(string car_id) {

	delete_car_from_database(_car_dict[car_id]);
	_car_dict.erase(car_id);
}

map<string, string>& routing_with_groups(const vector<vector<reference_wrapper<Car>>>& route_groups, map<string, string>& routes_dict) {
	// TODO: threading
	for (const vector<reference_wrapper<Car>> &route_group : route_groups) {
		map<string, vector<Node_in_Path>> result = routing(route_group);

		for (const pair < string, vector<Node_in_Path>>& result_data : result) {
			const string& car_id = result_data.first;
			const vector<Node_in_Path>& path_data = result_data.second;

			string turning_str = "";
			for (Node_in_Path node_in_path_data : path_data) {
				turning_str += node_in_path_data.turning;
			}
			_car_dict[car_id].path_data = path_data;

			routes_dict[car_id] = turning_str;
		}
	}

	return routes_dict;
}


map<string, vector<Node_in_Path>> routing(const vector<reference_wrapper<Car>>& route_group) {

	map<string, vector<Node_in_Path>> route_record;

	for (Car& car : route_group) {
		// Dijkstra's Algorithm

		// Variables
		map<Node_ID, Node_Record> nodes_arrival_time_data;
		const Coord& dst_coord = car.dst_coord;
		// Initualization
		Node_ID src_node(car.src_coord, car.direction_of_src_intersection);
		Node_Record src_record(true, (0 + car.time_offset_step));
		nodes_arrival_time_data[src_node] = src_record;  // Starting with an offset, last_node, recordings

		vector<Node_ID> visited_nodes;		// Record the visisted node

		priority_queue<Node_in_Heap, vector<Node_in_Heap>, Compare_AT > unvisited_queue;

		// Push the src node into the queue
		Node_in_Heap src_node_in_heap(nodes_arrival_time_data[src_node].arrival_time_stamp, src_node, car.position_at_offset);
		unvisited_queue.push(src_node_in_heap);

		Node_ID dst_node;

		// Routing
		while (unvisited_queue.size() > 0) {
			Node_in_Heap node_in_heap = unvisited_queue.top();
			unvisited_queue.pop();

			uint16_t current_arrival_time = node_in_heap.current_arrival_time;
			Node_ID current_node = node_in_heap.current_node;
			double position_at_offset = node_in_heap.position_at_offset;

			// Skip if the node is visited, prevent multiple push into the heap
			if (find(visited_nodes.begin(), visited_nodes.end(), current_node) != visited_nodes.end()){
				continue;
			}

			// Mark current node as "visisted"
			visited_nodes.push_back(current_node);

			Coord intersection_id = get<0>(current_node);
			uint8_t intersection_dir = get<1>(current_node);
			
			// Terminate when finding shortest path
			if (intersection_id == car.dst_coord) {
				car.traveling_time = double(current_arrival_time) * _schedule_period + position_at_offset / _V_MAX + double(_TOTAL_LEN) / _V_MAX; // additional time for car to leave sumo
				dst_node = current_node;
				break;
			}

			// Get information from database
			Intersection& intersection = get_intersection(current_arrival_time, intersection_id);

			// Decide the turnings
			map<char, Node_ID> available_turnings_and_out_direction = decide_available_turnings(intersection_id, intersection_dir, dst_coord, 0);

			for (pair<char, Node_ID> const& data_pair : available_turnings_and_out_direction) {
				char turning = data_pair.first;
				Node_ID node_id = data_pair.second;

				// Recording the states for final path
				vector<Car_in_Node_Record> recordings;

				car.lane = intersection_dir * LANE_NUM_PER_DIRECTION;
				car.current_turn = turning;
				car.lane = intersection.advise_lane(car);
				car.position = position_at_offset;
				car.update_dst_lane_and_data();

				// Determine the time arrive in Grouping Zone
				uint16_t time_in_GZ = current_arrival_time;
				while (position_at_offset > _GZ_BZ_CCZ_len) {
					// Record the path for final path retrieval
					// ###############################
					car.position = position_at_offset;
					Car_in_database record_car_advising = car;
					recordings.push_back( Car_in_Node_Record(time_in_GZ, intersection_id, "lane_advising", record_car_advising) );
					// ###############################

					time_in_GZ += 1;
					position_at_offset -= (double(_schedule_period) * _V_MAX);
				}

				Intersection& intersection_GZ = get_intersection(time_in_GZ, intersection_id);
				tuple<bool, double>result = intersection_GZ.is_GZ_full(car, position_at_offset);
				position_at_offset = get<1>(result);
				while (get<0>(result) == false) {
					// The intersection is full

					// Record the path for final path retrieval
					// ###############################
					car.position = position_at_offset;
					Car_in_database record_car_advising = car;
					recordings.push_back(Car_in_Node_Record(time_in_GZ, intersection_id, "lane_advising", record_car_advising));
					// ###############################

					time_in_GZ += 1;
					intersection_GZ = get_intersection(time_in_GZ, intersection_id);
					result = intersection_GZ.is_GZ_full(car, position_at_offset);
					position_at_offset = get<1>(result);
				}

				car.position = position_at_offset;
				double car_exiting_time = intersection_GZ.scheduling(car);

				while (car_exiting_time == SCHEDULE_POSPONDED || get<0>(result) == false) {
					// The scheduling is prosponed due to spillback

					// Record the path for final path retrieval
					// ###############################
					Car_in_database record_car_advising = car;
					recordings.push_back(Car_in_Node_Record(time_in_GZ, intersection_id, "lane_advising", record_car_advising));
					// ###############################

					time_in_GZ += 1;
					intersection_GZ = get_intersection(time_in_GZ, intersection_id);
					result = intersection_GZ.is_GZ_full(car, position_at_offset);

					if (get<0>(result) == true) {
						car.position = get<1>(result);
						car_exiting_time = intersection_GZ.scheduling(car);
					}
				}

				// Record the path for final path retrieval
				// ###############################
				Car_in_database record_car_scheduling = car;
				recordings.push_back(Car_in_Node_Record(time_in_GZ, intersection_id, "scheduling", record_car_scheduling));
				// ###############################

				uint16_t next_time_step = time_in_GZ + int(car_exiting_time / _schedule_period);
				double next_position_at_offset = _TOTAL_LEN - ((floor(car_exiting_time / _schedule_period) + 1) * _schedule_period - car_exiting_time) * _V_MAX;

				Node_ID next_node = node_id;

				if ( (nodes_arrival_time_data.find(next_node) == nodes_arrival_time_data.end()) 
					|| (nodes_arrival_time_data[next_node].arrival_time_stamp > next_time_step)) {
					Node_Record tmp_node_record(false, next_time_step);
					tmp_node_record.turning = turning;
					tmp_node_record.last_intersection_id = current_node;
					tmp_node_record.recordings = recordings;
					nodes_arrival_time_data[next_node] = tmp_node_record;

					Node_in_Heap node_in_heap(next_time_step, next_node, next_position_at_offset);
					unvisited_queue.push(node_in_heap);
				}
			}
		}

		// Retrieve the paths
		
		vector<Node_in_Path> path_list;
		Node_Record node_data = nodes_arrival_time_data[dst_node];
		while (node_data.is_src == false) {
			path_list.insert(path_list.begin(), Node_in_Path(node_data.turning, node_data.recordings, node_data.arrival_time_stamp));
			node_data = nodes_arrival_time_data[node_data.last_intersection_id];
			cout << car.id << endl;
		}

		route_record[car.id] = path_list;

		add_car_to_database(car, path_list);
	}

	return route_record;
}

void routing_in_thread(vector<reference_wrapper<Car>>* route_group_ptr) {
	routing(*route_group_ptr);
}

map<char, Node_ID> decide_available_turnings(Coord src_coord, uint8_t src_intersection_direction, Coord dst_coord, uint16_t additional_search_range) {
	// additional_search_range : additional intersection number to be searched
	// Value of the dict : id of next intersection, the direction of next intersection
	map<char, Node_ID> available_turnings_and_out_direction;
	if (src_intersection_direction == 0) {
		if (get<0>(dst_coord) - get<0>(src_coord) > -additional_search_range) {
			if (get<0>(src_coord) < _grid_size || get<1>(src_coord) == get<1>(dst_coord)) {
				available_turnings_and_out_direction['R'] = Node_ID(Coord(get<0>(src_coord) + 1, get<1>(src_coord)), 3);
			}
		}

		if (get<0>(src_coord) - get<0>(dst_coord) > -additional_search_range) {
			if (get<0>(src_coord) > 1 || get<1>(src_coord) == get<1>(dst_coord)) {
				available_turnings_and_out_direction['L'] = Node_ID(Coord(get<0>(src_coord) - 1, get<1>(src_coord)), 1);
			}
		}

		if (get<1>(dst_coord) - get<1>(src_coord) > -additional_search_range) {
			if (get<1>(src_coord) < _grid_size || get<0>(src_coord) == get<0>(dst_coord)) {
				available_turnings_and_out_direction['S'] = Node_ID(Coord(get<0>(src_coord), get<1>(src_coord) + 1), 0);
			}
		}
	}
	else if (src_intersection_direction == 1) {
		if (get<1>(dst_coord) - get<1>(src_coord) > -additional_search_range) {
			if (get<1>(src_coord) < _grid_size || get<0>(src_coord) == get<0>(dst_coord)) {
				available_turnings_and_out_direction['R'] = Node_ID(Coord(get<0>(src_coord), get<1>(src_coord) + 1), 0);
			}
		}
		if (get<1>(src_coord) - get<1>(dst_coord) > -additional_search_range) {
			if (get<1>(src_coord) > 1 || get<0>(src_coord) == get<0>(dst_coord)) {
				available_turnings_and_out_direction['L'] = Node_ID(Coord(get<0>(src_coord), get<1>(src_coord) - 1), 2);
			}
		}
		if (get<0>(src_coord) - get<0>(dst_coord) > -additional_search_range) {
			if (get<0>(src_coord) > 1 || get<1>(src_coord) == get<1>(dst_coord)) {
				available_turnings_and_out_direction['S'] = Node_ID(Coord(get<0>(src_coord) - 1, get<1>(src_coord)), 1);
			}
		}
	}
	else if (src_intersection_direction == 2) {
		if (get<0>(src_coord) - get<0>(dst_coord) > -additional_search_range) {
			if (get<0>(src_coord) > 1 || get<1>(src_coord) == get<1>(dst_coord)) {
				available_turnings_and_out_direction['R'] = Node_ID(Coord(get<0>(src_coord) - 1, get<1>(src_coord)), 1);
			}
		}
		if (get<0>(dst_coord) - get<0>(src_coord) > -additional_search_range) {
			if (get<0>(src_coord) < _grid_size || get<1>(src_coord) == get<1>(dst_coord)) {
				available_turnings_and_out_direction['L'] = Node_ID(Coord(get<0>(src_coord) + 1, get<1>(src_coord)), 3);
			}
		}
		if (get<1>(src_coord) - get<1>(dst_coord) > -additional_search_range) {
			if (get<1>(src_coord) > 1 || get<0>(src_coord) == get<0>(dst_coord)) {
				available_turnings_and_out_direction['S'] = Node_ID(Coord(get<0>(src_coord), get<1>(src_coord) - 1), 2);
			}
		}
	}
	else if (src_intersection_direction == 3) {
		if (get<1>(src_coord) - get<1>(dst_coord) > -additional_search_range) {
			if (get<1>(src_coord) > 1 || get<0>(src_coord) == get<0>(dst_coord)) {
				available_turnings_and_out_direction['R'] = Node_ID(Coord(get<0>(src_coord), get<1>(src_coord) - 1), 2);
			}
		}
		if (get<1>(dst_coord) - get<1>(src_coord) > -additional_search_range) {
			if (get<1>(src_coord) < _grid_size || get<0>(src_coord) == get<0>(dst_coord)) {
				available_turnings_and_out_direction['L'] = Node_ID(Coord(get<0>(src_coord), get<1>(src_coord) + 1), 0);
			}
		}
		if (get<0>(dst_coord) - get<0>(src_coord) > -additional_search_range) {
			if (get<0>(src_coord) < _grid_size || get<1>(src_coord) == get<1>(dst_coord)) {
				available_turnings_and_out_direction['S'] = Node_ID(Coord(get<0>(src_coord) + 1, get<1>(src_coord)), 3);
			}
		}
	}

	return available_turnings_and_out_direction; // Key : turnings, Values : out_direction
}

void add_car_to_database(Car& target_car, const vector<Node_in_Path>& path_list) {
	// TODO: write lock
	
	// Put all to-write records together
	vector<reference_wrapper<const Car_in_Node_Record>> recordings;
	for (const Node_in_Path& node_in_path_record : path_list) {
		for (const Car_in_Node_Record& record : node_in_path_record.recordings) {
			recordings.push_back(record);
		}
	}

	const Car_in_Node_Record* pre_record = nullptr;

	for (const Car_in_Node_Record& record : recordings) {
		const uint16_t& time = record.time_stamp;
		const Coord& intersection_id = record.last_intersection_id;
		const string& state = record.car_state;
		const Car_in_database& car = record.car_in_database;
		
		Intersection& intersection = get_intersection(time, intersection_id);

		if (state.compare("lane_advising")) {
			intersection.add_advising_car(car);
			tuple<string, reference_wrapper<Intersection>> to_save_tuple("lane_advising", intersection);
			target_car.records_intersection_in_database.push_back(to_save_tuple);
		}
		else if (state.compare("lane_advising")) {
			intersection.add_scheduling_cars(car);
			tuple<string, reference_wrapper<Intersection>> to_save_tuple("scheduling", intersection);
			target_car.records_intersection_in_database.push_back(to_save_tuple);
		}

		// See if the record change to next intersection: add scheduled cars
		if (pre_record != nullptr && intersection_id != pre_record->last_intersection_id) {
			const uint16_t& pre_time = pre_record->time_stamp;
			Car_in_database saving_car = pre_record->car_in_database;

			for (uint16_t time_idx = pre_time + 1; time_idx < time; time_idx++) {
				saving_car.OT -= _schedule_period;

				if (saving_car.OT + saving_car.D > 0) {
					Intersection& intersection_to_save = get_intersection(time_idx, intersection_id);
					intersection_to_save.add_sched_car(saving_car);
					tuple<string, reference_wrapper<Intersection>> to_save_tuple("scheduled", intersection_to_save);
					target_car.records_intersection_in_database.push_back(to_save_tuple);
				}
			}
		}
		pre_record = &record;
	}

	// Add the scheduled car before exiting to the database
	const uint16_t& exiting_time = path_list.back().time;
	const uint16_t& pre_time = (pre_record->time_stamp);
	const Coord& intersection_id = pre_record->last_intersection_id;
	Car_in_database saving_car = pre_record->car_in_database;

	for (uint16_t time_idx = pre_time + 1; time_idx < exiting_time; time_idx++) {
		saving_car.OT -= _schedule_period;

		if (saving_car.OT + saving_car.D > 0) {
			Intersection& intersection_to_save = get_intersection(time_idx, intersection_id);
			intersection_to_save.add_sched_car(saving_car);
			tuple<string, reference_wrapper<Intersection>> to_save_tuple("scheduled", intersection_to_save);
			target_car.records_intersection_in_database.push_back(to_save_tuple);
		}
	}

}

void testQ() {

}
