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
		_database.erase(_database.begin(), _database.begin()+_routing_period_num);
	}
}

Intersection& get_intersection(const int current_arrival_time, const Coord &intersection_id) {
	while (current_arrival_time >= int(_database.size())) {
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

	for ( tuple<string, Intersection> &record : car.records_intersection_in_database) {
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

void routing_with_groups(vector<vector<reference_wrapper<Car>>> route_groups, map<string, string> routes_dict) {
	
	for (vector<reference_wrapper<Car>> &route_group : route_groups) {
		routing(route_group);
	}

	// TODO: construct result (path)
}

class Node_Record {
public:
	bool is_src = false;
	uint16_t arrival_time_stamp = 0;		// "Database time" that the car arrives
	Coord last_intersection_id = Coord(0,0);
	char turning = 'S';
	// Recordings

	Node_Record() {}
	Node_Record(bool in_is_src, uint16_t arrival_time_stamp): is_src(in_is_src), arrival_time_stamp(arrival_time_stamp){}
};

typedef tuple<Coord, uint8_t> Node_ID;
class Node_in_Heap {
public:
	uint16_t current_arrival_time = 0;
	Node_ID current_node;
	double position_at_offset = 0;

	Node_in_Heap(){}
	Node_in_Heap(uint16_t arrival_time, Node_ID node, double pos_offset): current_arrival_time(arrival_time), current_node(node), position_at_offset(pos_offset){}
};
struct Compare_AT {
	bool operator()(Node_in_Heap const &node1, Node_in_Heap const &node2) {
		return node1.current_arrival_time > node2.current_arrival_time;
	}
};

map<char, Node_ID> decide_available_turnings(Coord src_coord, uint8_t src_intersection_direction, Coord dst_coord, uint16_t additional_search_range);

// TODO: modify the input type here
void routing(vector<reference_wrapper<Car>>& route_group) {

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

				car.lane = intersection_dir * LANE_NUM_PER_DIRECTION;
				car.current_turn = turning;
				car.lane = intersection.advise_lane(car);
				car.position = position_at_offset;
				car.update_dst_lane_and_data();
			}
		}
	}
}

map<char, Node_ID> decide_available_turnings(Coord src_coord, uint8_t src_intersection_direction, Coord dst_coord, uint16_t additional_search_range) {
	// additional_search_range : additional intersection number to be searched
	// Value of the dict : id of next intersection, the direction of next intersection
	map<char, Node_ID> available_turnings_and_out_direction;
	if (src_intersection_direction == 0) {
		if (get<0>(dst_coord) - get<0>(src_coord) > -additional_search_range) {
			if (get<0>(src_coord) < _grid_size or get<1>(src_coord) == get<1>(dst_coord)) {
				available_turnings_and_out_direction['R'] = Node_ID(Coord(get<0>(src_coord) + 1, get<1>(src_coord)), 3);
			}
		}

		if (get<0>(src_coord) - get<0>(dst_coord) > -additional_search_range) {
			if (get<0>(src_coord) > 1 or get<1>(src_coord) == get<1>(dst_coord)) {
				available_turnings_and_out_direction['L'] = Node_ID(Coord(get<0>(src_coord) - 1, get<1>(src_coord)), 1);
			}
		}

		if (get<1>(dst_coord) - get<1>(src_coord) > -additional_search_range) {
			if (get<1>(src_coord) < _grid_size or get<0>(src_coord) == get<0>(dst_coord)) {
				available_turnings_and_out_direction['S'] = Node_ID(Coord(get<0>(src_coord), get<1>(src_coord) + 1), 0);
			}
		}
	}
	else if (src_intersection_direction == 1) {
		if (get<1>(dst_coord) - get<1>(src_coord) > -additional_search_range) {
			if (get<1>(src_coord) < _grid_size or get<0>(src_coord) == get<0>(dst_coord)) {
				available_turnings_and_out_direction['R'] = Node_ID(Coord(get<0>(src_coord), get<1>(src_coord) + 1), 0);
			}
		}
		if (get<1>(src_coord) - get<1>(dst_coord) > -additional_search_range) {
			if (get<1>(src_coord) > 1 or get<0>(src_coord) == get<0>(dst_coord)) {
				available_turnings_and_out_direction['L'] = Node_ID(Coord(get<0>(src_coord), get<1>(src_coord) - 1), 2);
			}
		}
		if (get<0>(src_coord) - get<0>(dst_coord) > -additional_search_range) {
			if (get<0>(src_coord) > 1 or get<1>(src_coord) == get<1>(dst_coord)) {
				available_turnings_and_out_direction['S'] = Node_ID(Coord(get<0>(src_coord) - 1, get<1>(src_coord)), 1);
			}
		}
	}
	else if (src_intersection_direction == 2) {
		if (get<0>(src_coord) - get<0>(dst_coord) > -additional_search_range) {
			if (get<0>(src_coord) > 1 or get<1>(src_coord) == get<1>(dst_coord)) {
				available_turnings_and_out_direction['R'] = Node_ID(Coord(get<0>(src_coord) - 1, get<1>(src_coord)), 1);
			}
		}
		if (get<0>(dst_coord) - get<0>(src_coord) > -additional_search_range) {
			if (get<0>(src_coord) < _grid_size or get<1>(src_coord) == get<1>(dst_coord)) {
				available_turnings_and_out_direction['L'] = Node_ID(Coord(get<0>(src_coord) + 1, get<1>(src_coord)), 3);
			}
		}
		if (get<1>(src_coord) - get<1>(dst_coord) > -additional_search_range) {
			if (get<1>(src_coord) > 1 or get<0>(src_coord) == get<0>(dst_coord)) {
				available_turnings_and_out_direction['S'] = Node_ID(Coord(get<0>(src_coord), get<1>(src_coord) - 1), 2);
			}
		}
		else if (src_intersection_direction == 3) {
			if (get<1>(src_coord) - get<1>(dst_coord) > -additional_search_range) {
				if (get<1>(src_coord) > 1 or get<0>(src_coord) == get<0>(dst_coord)) {
					available_turnings_and_out_direction['R'] = Node_ID(Coord(get<0>(src_coord), get<1>(src_coord) - 1), 2);
				}
			}
			if (get<1>(dst_coord) - get<1>(src_coord) > -additional_search_range) {
				if (get<1>(src_coord) < _grid_size or get<0>(src_coord) == get<0>(dst_coord)) {
					available_turnings_and_out_direction['L'] = Node_ID(Coord(get<0>(src_coord), get<1>(src_coord) + 1), 0);
				}
			}
			if (get<0>(dst_coord) - get<0>(src_coord) > -additional_search_range) {
				if (get<0>(src_coord) < _grid_size or get<1>(src_coord) == get<1>(dst_coord)) {
					available_turnings_and_out_direction['S'] = Node_ID(Coord(get<0>(src_coord) + 1, get<1>(src_coord)), 3);
				}
			}
		}
	}

	return available_turnings_and_out_direction; // Key : turnings, Values : out_direction
}


void testQ() {

}
