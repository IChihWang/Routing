#include "global.h"
#include "thread_worker.h"

#include <sstream>
#include <cstdlib>
#include <iostream>
#include <cassert>
#include <thread>
#include <set>
using namespace std;


uint8_t _CHOOSE_CAR_OPTION;
uint8_t _TOP_N_CONGESTED;
vector < map< Coord, Intersection* >* > _database;
map<string, Car> _car_dict;
vector<pair<int32_t, Intersection*>> _top_congested_intersections;

shared_mutex wlock_mutex_affected_intersections;
set<pair<uint16_t, Intersection*>> affected_intersections;


/* Handling database */

void create_grid_network() {
	for (int idx = 0; idx < DEFAULT_DATA_LENGTH; idx++) {
		add_time_step(idx*_schedule_period);
	}
}

void add_time_step(double time_stamp) {
	map< Coord, Intersection* >* intersection_map = new map< Coord, Intersection* >;
	for (int i = 1; i <= _grid_size; i++) {
		for (int j = 1; j <= _grid_size; j++) {
			Coord coordinate(i, j);
			Intersection* intersection = new Intersection(coordinate, time_stamp);
			(*intersection_map)[coordinate] = intersection;
		}
	}
	_database.push_back(intersection_map);

	// Connect the intersections
	map< Coord, Intersection* >* map_in_database = _database.back();
	connect_intersections(*map_in_database);
}

void connect_intersections(map< Coord, Intersection* >& intersection_map) {
	for (int i = 1; i <= _grid_size; i++) {
		for (int j = 1; j <= _grid_size; j++) {
			Coord coordinate(i, j);
			if (i <= _grid_size - 1) {
				Coord target_coordinate(i + 1, j);
				(*(intersection_map[coordinate])).connect(1, (*(intersection_map[target_coordinate])), 3);
			}
			if (j <= _grid_size - 1) {
				Coord target_coordinate(i, j + 1);
				(*(intersection_map[coordinate])).connect(2, (*(intersection_map[target_coordinate])), 0);
			}
		}
	}
}

void move_a_time_step() {
	while (_database.size() > 0) {
		map< Coord, Intersection* >* intersection_map = _database[0];
		if ((*intersection_map)[Coord(1,1)]->time_stamp < _NOW_SIMU_TIME + _routing_period) {
			for (auto& [intersection_id, intersection_ptr] : (*intersection_map)) {
				intersection_ptr->my_own_destructure();
				delete intersection_ptr;
			}
			delete intersection_map;
			_database.erase(_database.begin());
		}
		else {
			break;
		}
	}

	_top_congested_intersections.erase(
		std::remove_if(
			_top_congested_intersections.begin(),
			_top_congested_intersections.end(),
			[](pair<int32_t, Intersection*> const& p) { return p.second->time_stamp < _NOW_SIMU_TIME+_routing_period; }
		),
		_top_congested_intersections.end()
	);
	/*
	for (uint16_t i = 0; i < _routing_period_num; i++) {
		map< Coord, Intersection* >* intersection_map = _database[0];
		for (auto& [intersection_id, intersection_ptr] : (*intersection_map)) {
			intersection_ptr->my_own_destructure();
			delete intersection_ptr;
		}
		delete intersection_map;
		_database.erase(_database.begin());
	}

	for (pair<int32_t, Intersection*>& item : _top_congested_intersections) {
		item.first -= _routing_period_num;
	}

	_top_congested_intersections.erase(
		std::remove_if(
			_top_congested_intersections.begin(),
			_top_congested_intersections.end(),
			[](pair<int32_t, Intersection*> const& p) { return p.first <= 0; }
		),
		_top_congested_intersections.end()
	);

	*/
}

shared_mutex database_mutex;
Intersection& get_intersection(const uint16_t current_arrival_time, const Coord &intersection_id) {
	while (current_arrival_time >= int(_database.size())) {
		lock_guard<shared_mutex> database_write_lock(database_mutex);
		add_time_step(_NOW_SIMU_TIME + _database.size()*_schedule_period);
	}

	map< Coord, Intersection* >* intersection_map = _database[current_arrival_time];
	Intersection* intersection = (*intersection_map)[intersection_id];
	return *intersection;
}

Car_in_Node_Record::Car_in_Node_Record(const Car_in_Node_Record& target_car) {
	time_stamp = target_car.time_stamp;				// "Database time" that the car arrives
	last_intersection_id = target_car.last_intersection_id;
	car_state = target_car.car_state;
	car_in_database = target_car.car_in_database;
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
	// Important: a car shouldn't appear in two different groups at the same time!!!!!!!!!!!!

	/*
	* _CHOOSE_CAR_OPTION == 0: BFS on rescheduling/  ideal and fast
	* _CHOOSE_CAR_OPTION == 1: without BFS on rescheduling/ Not ideal but fast
	* _CHOOSE_CAR_OPTION == 2: Round-robin top-N cars on rescheduling (one thread on rescheduling)/ ideal but slow
	* _CHOOSE_CAR_OPTION == 3: Round-robin all cars on rescheduling (one thread on rescheduling)/ most ideal but very slow
	* _CHOOSE_CAR_OPTION == 4: No rescheduling
	*/

	for (int i = 0; i < _thread_num; i++) {
		results.push_back(vector<reference_wrapper<Car>>());
	}


	if (_CHOOSE_CAR_OPTION == 0) {
		// Find the correlatioin between affected intersections (Whether they share same CAV)
		vector<pair<uint32_t, Intersection*>> not_yet_searched_list(_top_congested_intersections.begin(), _top_congested_intersections.end());
		vector<pair<uint32_t, Intersection*>> searched_list;
		vector<pair<uint32_t, Intersection*>> searching_list;
		vector<string> car_added_list;

		while (not_yet_searched_list.size() > 0) {
			// Find the thread with minimum size to store the current tree
			vector<reference_wrapper<Car>>& target_result = *min_element(results.begin(), results.end(),
				[](const vector<reference_wrapper<Car>>& a, const vector<reference_wrapper<Car>>& b) -> bool
				{
					return a.size() < b.size();
				});

			// Start constructing the tree (BFS)
			searching_list.push_back(not_yet_searched_list[0]);
			not_yet_searched_list.erase(not_yet_searched_list.begin());

			while (searching_list.size() > 0) {
				pair<uint32_t, Intersection*>& visiting_intersection = searching_list[0];
				searching_list.erase(searching_list.begin());

				if (find(searched_list.begin(), searched_list.end(), visiting_intersection) != searched_list.end()) {
					// Visisted, so skip
					continue;
				}

				searched_list.push_back(visiting_intersection);
				Intersection* visiting_intersection_ptr = visiting_intersection.second;
				vector<uint32_t> to_be_deleted_idx;
				for (uint16_t idx = 0; idx < not_yet_searched_list.size(); idx++) {
					const pair<uint32_t, Intersection*>& checking_intersection = not_yet_searched_list[idx];
					uint32_t timestamp = checking_intersection.first;
					Intersection* checking_intersection_ptr = checking_intersection.second;

					for (auto item : *(visiting_intersection_ptr->scheduling_cars)) {
						string car_id = item.first;
						if ((checking_intersection_ptr->scheduling_cars)->find(car_id) != (checking_intersection_ptr->sched_cars)->end()) {
							// Find car in another intersection, correlated
							searching_list.push_back(checking_intersection);
							to_be_deleted_idx.push_back(idx);
							break;
						}
					}
				}

				// Remove element from the back
				for (int idx = to_be_deleted_idx.size() - 1; idx >= 0; idx--) {
					not_yet_searched_list.erase(not_yet_searched_list.begin() + to_be_deleted_idx[idx]);
				}

				// Record the car results
				for (auto item : *(visiting_intersection_ptr->scheduling_cars)) {
					string car_id = item.first;
					if (_car_dict[car_id].state == "OLD" || _car_dict[car_id].state == "NEW") {
						if (find(car_added_list.begin(), car_added_list.end(), car_id) == car_added_list.end()) {
							target_result.push_back(_car_dict[car_id]);
							car_added_list.push_back(car_id);
						}
					}
				}
			}
		}

	}
	else if (_CHOOSE_CAR_OPTION == 1) {
		set<string> cars_to_add;
		for (const pair<uint32_t, Intersection*> item : _top_congested_intersections) {
			Intersection* intersection_ptr = item.second;
			for (auto car_item : *(intersection_ptr->scheduling_cars)) {
				const string& car_id = car_item.first;
				if (_car_dict[car_id].state.compare("OLD") == 0 || _car_dict[car_id].state.compare("NEW") == 0) {
					cars_to_add.insert(car_id);
				}

			}
		}

		for (const string& car_id : cars_to_add) {
			// Find the thread with minimum size to store the current tree
			vector<reference_wrapper<Car>>& target_result = *min_element(results.begin(), results.end(),
				[](const vector<reference_wrapper<Car>>& a, const vector<reference_wrapper<Car>>& b) -> bool
				{
					return a.size() < b.size();
				});

			target_result.push_back(_car_dict[car_id]);
		}
	}
	else if (_CHOOSE_CAR_OPTION == 2) {

		for (const pair<uint32_t, Intersection*> item : _top_congested_intersections) {
			vector<reference_wrapper<Car>>& target_result = results[0];

			Intersection* intersection_ptr = item.second;
			for (auto car_item : *(intersection_ptr->scheduling_cars)) {
				string car_id = car_item.first;
				target_result.push_back(_car_dict[car_id]);
			}
		}
	}
	else if (_CHOOSE_CAR_OPTION == 3) {

		for (const pair<string, Car> car_item : _car_dict) {
			if (find(new_car_ids.begin(), new_car_ids.end(), car_item.first) != new_car_ids.end()) {
				// New car, skip
				continue;
			}

			vector<reference_wrapper<Car>>& target_result = results[0];
			string car_id = car_item.first;
			target_result.push_back(_car_dict[car_id]);
		}
	}
	else if (_CHOOSE_CAR_OPTION == 4) {
		// Do nothing with the old cars
	}
	
	/*
	cout << "=====" << endl;
	for (auto car_vec : results) {
		cout << " " << car_vec.size() << "   ";
		for (Car& car : car_vec) {
			cout << car.id << " " << _car_dict[car.id].state << " | ";
		}
		cout << endl;
	}
	cout << endl;
	*/


	// Handling the new cars
	auto min_max_result = minmax_element(results.begin(), results.end(), 
		[](const vector<reference_wrapper<Car>>& a, const vector<reference_wrapper<Car>>& b) -> bool
		{
			return a.size() < b.size();
		});
	vector<reference_wrapper<Car>>* min_group = &(*min_max_result.first);
	vector<reference_wrapper<Car>>* max_group = &(*min_max_result.second);
	for (string car_id : new_car_ids) {
		min_group->push_back(_car_dict[car_id]);
		if ((int)min_group->size() > (int)max_group->size() - 3) {	// 3 is a temporary number to control the load balance
			// Re-sort the group

			auto min_max_result = minmax_element(results.begin(), results.end(),
				[](const vector<reference_wrapper<Car>>& a, const vector<reference_wrapper<Car>>& b) -> bool
				{
					return a.size() < b.size();
				});
			min_group = &(*min_max_result.first);
			max_group = &(*min_max_result.second);
		}
	}

	for (auto car_vec : results) {
		cout << " " << car_vec.size() << " ";
	}
	cout << endl;

	/*
	* Looping through each bucket
	int process_idx = 0;
	for (string car_id : new_car_ids) {
		process_idx++;
		process_idx %= _thread_num;
		results[process_idx].push_back(_car_dict[car_id]);
	}
	*/

	// Remove Cars from the database for new routes
	for (vector<reference_wrapper<Car>> car_group : results) {
		for (Car &car: car_group) {
			delete_car_from_database(car);
		}
	}

	return results;
}

void delete_car_from_database(Car &car) {

	vector<Intersection*> time_out_inter_ptr;
	double car_abs_time_offset =  _NOW_SIMU_TIME + car.time_offset_step * _schedule_period - 0.1;
	for (const pair<Intersection*, pair<string, double>>& record : car.records_intersection_in_database) {
		const string& type = get<0>(get<1>(record));
		const double& time_stamp = get<1>(get<1>(record));
		Intersection* intersection = get<0>(record);

		if (time_stamp >= _NOW_SIMU_TIME and intersection->time_stamp >= car_abs_time_offset) {
			intersection->delete_car_from_intersection(car, type);
		}
	}
}

void delete_car_from_database_id(string car_id) {

	delete_car_from_database(_car_dict[car_id]);
	_car_dict.erase(car_id);
}

/*
map<string, string>& routing_with_groups(const vector<vector<reference_wrapper<Car>>>& route_groups, map<string, string>& routes_dict) {
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
*/

map<string, vector<Node_in_Path>> routing(const vector<reference_wrapper<Car>>& route_group, set< pair<uint16_t, Intersection*> >& thread_affected_intersections) {
	thread_affected_intersections.clear();
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
			double node_position_at_offset = node_in_heap.position_at_offset;

			car.is_spillback = false;
			car.is_spillback_strict = false;


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
				car.traveling_time = _NOW_SIMU_TIME + double(current_arrival_time) * _schedule_period + (500 - (_TOTAL_LEN - node_position_at_offset)) / _V_MAX; // additional time for car to leave sumo
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
				double position_at_offset = node_position_at_offset;

				// Recording the states for final path
				vector<Car_in_Node_Record> recordings;

				car.lane = intersection_dir * LANE_NUM_PER_DIRECTION;
				car.set_turn(turning);
				car.lane = intersection.advise_lane(car);
				car.position = position_at_offset;
				car.update_dst_lane_and_data();

				if (car.id == "car_153") {
					cout << " =============================== 111------1 " << endl;
					cout << car.id << "      " << position_at_offset << " " << turning << " " << _GZ_BZ_CCZ_len << " " << _NOW_SIMU_TIME + current_arrival_time * _schedule_period << endl;
					cout << endl << "=============================== 111-------1 " << endl << endl;
					cout << "Zero timestamp: " << (*(_database[0]))[Coord(1, 1)]->time_stamp << " Now from SUMO:" << _NOW_SIMU_TIME << endl;
					for (auto& veccc : _database) {
						cout << "Timestamps: " << (*veccc)[Coord(1, 1)]->time_stamp << endl;
					}
				}

				// Determine the time arrive in Grouping Zone
				uint16_t time_in_GZ = current_arrival_time;
				while (position_at_offset > _GZ_BZ_CCZ_len) {


					// Record the path for final path retrieval
					// ###############################
					car.position = position_at_offset;
					Car_in_database record_car_advising = car;
					Car_in_Node_Record tmp_record(time_in_GZ, intersection_id, "lane_advising", record_car_advising);
					recordings.push_back(tmp_record);
					// ###############################

					time_in_GZ += 1;

					position_at_offset -= (double(_schedule_period) * _V_MAX);

					if (car.id == "car_153") {
						cout << " =============================== 111 " << endl;
						cout << car.id << "    " << time_in_GZ << " " << turning << " " << position_at_offset << " " << _NOW_SIMU_TIME + time_in_GZ * _schedule_period << endl;
						cout << endl << "=============================== 111 " << endl << endl;
					}
				}

				Intersection* intersection_GZ_ptr = &(get_intersection(time_in_GZ, intersection_id));
				tuple<bool, double>result = intersection_GZ_ptr->is_GZ_full(car, position_at_offset);

				if (car.id == "car_153") {
					cout << " =============================== 222 " << endl;
					cout << car.id << " " << turning << " " << intersection_GZ_ptr->id_str << " " << _NOW_SIMU_TIME + time_in_GZ * _schedule_period << endl;
					cout << "time_stamp: " << intersection_GZ_ptr->time_stamp << " time step_shift: " << time_in_GZ << " " << _schedule_period << endl;
					cout << "-------------------" << endl;
					for (auto [car_id, local_car] : *(intersection_GZ_ptr->sched_cars)) {
						cout << car_id << " | ";
					}
					cout << endl << "-------------------" << endl;
					for (auto [car_id, local_car] : *(intersection_GZ_ptr->scheduling_cars)) {
						cout << car_id << " | ";
					}
					cout << endl << "-------------------" << endl;
					for (auto [car_id, local_car] : *(intersection_GZ_ptr->advising_car)) {
						cout << car_id << " " << local_car.is_spillback_strict << " | ";
					}
					cout << endl << "=============================== 222 " << endl << endl;
				}

				position_at_offset = get<1>(result);
				while (get<0>(result) == false) {
					// The intersection is full

					// Record the path for final path retrieval
					// ###############################
					car.position = position_at_offset;
					Car_in_database record_car_advising = car;
					Car_in_Node_Record tmp_record(time_in_GZ, intersection_id, "lane_advising", record_car_advising);
					recordings.push_back(tmp_record);
					// ###############################

					time_in_GZ += 1;

					intersection_GZ_ptr = &(get_intersection(time_in_GZ, intersection_id));
					result = intersection_GZ_ptr->is_GZ_full(car, position_at_offset);

					if (car.id == "car_153") {
						cout << "============================ 222-2 " << endl;
						cout << car.id << " " << turning << " " << intersection_GZ_ptr->id_str << " " << _NOW_SIMU_TIME + time_in_GZ * _schedule_period << endl;
						cout << "time_stamp: " << intersection_GZ_ptr->time_stamp << " time step_shift: " << time_in_GZ << " " << _schedule_period << endl;
						cout << "-------------------" << endl;
						for (auto [car_id, local_car] : *(intersection_GZ_ptr->sched_cars)) {
							cout << car_id << " | ";
						}
						cout << endl << "-------------------" << endl;
						for (auto [car_id, local_car] : *(intersection_GZ_ptr->scheduling_cars)) {
							cout << car_id << " | ";
						}
						cout << endl << "-------------------" << endl;
						for (auto [car_id, local_car] : *(intersection_GZ_ptr->advising_car)) {
							cout << car_id << " " << local_car.is_spillback_strict << " | ";
						}
						cout << endl << "============================ 222-2 " << endl << endl;
					}

					position_at_offset = get<1>(result);
				}


				car.position = position_at_offset;
				double car_exiting_time = intersection_GZ_ptr->scheduling(car);

				if (car.id == "car_153") {
					cout << endl << "============================ 333" << endl;
					cout << car.id << " " << turning << " " << intersection_GZ_ptr->id_str << " " << _NOW_SIMU_TIME+ time_in_GZ*_schedule_period << endl;
					cout << "time_stamp: " << intersection_GZ_ptr->time_stamp << " time step_shift: " << time_in_GZ << " " << _schedule_period << endl;
					cout << car_exiting_time << endl;
					cout << "-------------------" << endl;
					for (auto [car_id, local_car] : *(intersection_GZ_ptr->sched_cars)) {
						cout << car_id << " " << local_car.D+local_car.OT << " | ";
					}
					cout << endl << "-------------------" << endl;
					for (auto [car_id, local_car] : *(intersection_GZ_ptr->scheduling_cars)) {
						cout << car_id << " " << local_car.D + local_car.OT << " | ";
					}
					cout << endl << "-------------------" << endl;
					for (auto [car_id, local_car] : *(intersection_GZ_ptr->advising_car)) {
						cout << car_id << " " << local_car.is_spillback_strict << " | ";
					}
					cout << endl << "============================ 333-2 " << endl << endl;
				}

				while (car_exiting_time == SCHEDULE_POSPONDED || get<0>(result) == false) {
					// The scheduling is prosponed due to spillback

					// Record the path for final path retrieval
					// ###############################
					Car_in_database record_car_advising = car;
					Car_in_Node_Record tmp_record(time_in_GZ, intersection_id, "lane_advising", record_car_advising);
					recordings.push_back(tmp_record);
					// ###############################

					time_in_GZ += 1;
					Intersection& intersection_GZ = get_intersection(time_in_GZ, intersection_id);
					result = intersection_GZ.is_GZ_full(car, position_at_offset);

					if (get<0>(result) == true) {
						car.position = get<1>(result);
						car_exiting_time = intersection_GZ.scheduling(car);

						if (car.id == "car_153") {
							cout << "=============================== 444" << endl;
							cout << car.id << " " << turning << " " << intersection_GZ_ptr->id_str << " " << _NOW_SIMU_TIME + time_in_GZ * _schedule_period << endl;
							cout << "time_stamp: " << intersection_GZ_ptr->time_stamp << endl;
							cout << "-------------------" << endl;
							for (auto [car_id, local_car] : *(intersection_GZ_ptr->sched_cars)) {
								cout << car_id << " | ";
							}
							cout << endl << "-------------------" << endl;
							for (auto [car_id, local_car] : *(intersection_GZ_ptr->scheduling_cars)) {
								cout << car_id << " | ";
							}
							cout << endl << "-------------------" << endl;
							for (auto [car_id, local_car] : *(intersection_GZ_ptr->advising_car)) {
								cout << car_id << " " << local_car.is_spillback_strict << " | ";
							}
							cout << endl << "============================ 444" << endl << endl;
						}
					}
				}

				// Record the path for final path retrieval
				// ###############################
				Car_in_database record_car_scheduling = car;
				Car_in_Node_Record tmp_record(time_in_GZ, intersection_id, "scheduling", record_car_scheduling);
				recordings.push_back(tmp_record);
				// ###############################

				// Add time in intersection (number by measuring the travel time in min trajectory)
				if (turning == 'L')
					car_exiting_time += 2.4;
				else if(turning == 'S')
					car_exiting_time += 2.4;
				else if (turning == 'R')
					car_exiting_time += 0.9;


				uint16_t next_time_step = time_in_GZ + ceil(car_exiting_time / _schedule_period);
				double next_position_at_offset = _TOTAL_LEN - (ceil(car_exiting_time / _schedule_period) * _schedule_period - car_exiting_time) * _V_MAX;

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

		if (nodes_arrival_time_data.find(dst_node) == nodes_arrival_time_data.end()) {
			cout << "No route is found for " << car.id << endl;
			cout << get<0>(_car_dict[car.id].src_coord) << "," << get<1>(_car_dict[car.id].src_coord) << " ";
			cout << get<0>(_car_dict[car.id].dst_coord) << "," << get<1>(_car_dict[car.id].dst_coord) << " ";
			cout << (int)_car_dict[car.id].direction_of_src_intersection << " ";
			cout << (int)_car_dict[car.id].time_offset_step << " ";
			cout << _car_dict[car.id].position_at_offset << endl;


			exit(-1);
		}
		Node_Record node_data = nodes_arrival_time_data[dst_node];
		while (node_data.is_src == false) {
			
			if (car.id == "car_153") {
				cout << car.id << " " << node_data.turning << " " << _NOW_SIMU_TIME + car.time_offset_step * _schedule_period << " " << get<0>(get<0>(node_data.last_intersection_id)) << "," << get<1>(get<0>(node_data.last_intersection_id)) << " " << (int)get<1>(node_data.last_intersection_id) << " " << _NOW_SIMU_TIME + node_data.arrival_time_stamp * _schedule_period << endl;
			}

			path_list.insert(path_list.begin(), Node_in_Path(node_data.turning, node_data.recordings, node_data.arrival_time_stamp));
			node_data = nodes_arrival_time_data[node_data.last_intersection_id];
		}

		route_record[car.id] = path_list;

		add_car_to_database(car, path_list, thread_affected_intersections);
	}

	return route_record;
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

void add_intersection_to_reschedule_list() {
	
	affected_intersections.clear();

	for (uint8_t thread_i = 0; thread_i < _thread_num; thread_i++) {
		set< pair<uint16_t, Intersection*> >& thread_affected_intersections = _thread_pool[thread_i]->thread_affected_intersections;
		affected_intersections.insert(thread_affected_intersections.begin(), thread_affected_intersections.end());
	}

	// /*
	vector<pair<uint16_t, Intersection*>> sorted_affected_intersections(affected_intersections.begin(), affected_intersections.end());
	sorted_affected_intersections.insert(sorted_affected_intersections.end(), _top_congested_intersections.begin(), _top_congested_intersections.end());
	sort(sorted_affected_intersections.begin(), sorted_affected_intersections.end(), [](pair<uint16_t, Intersection*> a, pair<uint16_t, Intersection*> b) -> bool {return (a.second)->get_car_num() > (b.second)->get_car_num(); });

	//_top_congested_intersections = vector<pair<int32_t, Intersection*>>(sorted_affected_intersections.begin(), sorted_affected_intersections.begin()+ _TOP_N_CONGESTED);
	_top_congested_intersections.clear();
	uint8_t count = 0;
	for (auto sorted_item : sorted_affected_intersections) {
		bool is_skip = false;
		for (auto item : _top_congested_intersections) {
			if (sorted_item.second->id_str == item.second->id_str && abs(sorted_item.first - item.first) < 2*double(_TOTAL_LEN)/_V_MAX){
				is_skip = true;
				break;
			}
		}
		
		if (!is_skip) {
			_top_congested_intersections.push_back(sorted_item);

			if (++count >= _TOP_N_CONGESTED)
				break;
		}
	}


	// */
	 /*
	for (auto item : affected_intersections) {
		const uint16_t time = item.first;
		Intersection* intersection_ptr = item.second;

		//cout << (intersection_ptr->scheduling_cars)->size() << endl;

		uint8_t insert_pos = 0;
		for (insert_pos; insert_pos < _top_congested_intersections.size(); insert_pos++) {
			Intersection* comparing_intersection_ptr = _top_congested_intersections[insert_pos].second;
			if (intersection_ptr->get_car_num() > comparing_intersection_ptr->get_car_num()) {
				break;
			}
		}

		if (insert_pos >= _TOP_N_CONGESTED) {
			continue;
		}
	
		_top_congested_intersections.insert(_top_congested_intersections.begin() + insert_pos, pair(time, intersection_ptr));
		while (_top_congested_intersections.size() > _TOP_N_CONGESTED) {
			_top_congested_intersections.pop_back();
		}
	}


	// */
}

void add_car_to_database(Car& target_car, const vector<Node_in_Path>& path_list, set< pair<uint16_t, Intersection*> >& thread_affected_intersections) {
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

		if (state.compare("lane_advising") == 0) {
			intersection.add_advising_car(car, target_car);
			Node_in_Car to_save_key(time, intersection_id);
			target_car.records_intersection_in_database[&intersection] = { "lane_advising", intersection.time_stamp };

			if (car.id == "car_153")
				cout << " lane_advising: " << intersection.time_stamp << " " << _NOW_SIMU_TIME + time * _schedule_period << " " << time << " " << intersection.id_str << " " << car.id << " " << car.D + car.OT << " " << car.OT << endl;
		}
		else if (state.compare("scheduling") == 0) {
			intersection.add_scheduling_cars(car, target_car);
			Node_in_Car to_save_key(time, intersection_id);
			target_car.records_intersection_in_database[&intersection] = {"scheduling", intersection.time_stamp };

			thread_affected_intersections.insert(pair(time, &intersection));

			if (car.id == "car_153")
				cout << " scheduling: " << intersection.time_stamp << " " << _NOW_SIMU_TIME + time * _schedule_period << " " << time << " " << intersection.id_str << " " << car.id << " " << car.D + car.OT << " " << car.OT << endl;
		}
		// See if the record change to next intersection: add scheduled cars
		if (pre_record != nullptr && intersection_id != pre_record->last_intersection_id) {
			const uint16_t& pre_time = pre_record->time_stamp;
			Car_in_database saving_car = pre_record->car_in_database;

			for (uint16_t time_idx = pre_time + 1; time_idx < time; time_idx++) {
				saving_car.OT -= _schedule_period;

				if (saving_car.OT + saving_car.D > 0) {
					Intersection& intersection_to_save = get_intersection(time_idx, pre_record->last_intersection_id);
					intersection_to_save.add_sched_car(saving_car, target_car);
					Node_in_Car to_save_key(time_idx, pre_record->last_intersection_id);
					target_car.records_intersection_in_database[&intersection_to_save] = { "scheduled", intersection_to_save.time_stamp };

					thread_affected_intersections.insert(pair(time_idx, &intersection_to_save));

					if (saving_car.id == "car_153")
						cout << " scheduled: " << intersection_to_save.time_stamp << " " << _NOW_SIMU_TIME + time_idx * _schedule_period << " " << time_idx << " " << intersection_to_save.id_str << " " << saving_car.id << " " << saving_car.D + saving_car.OT << " " << saving_car.OT << endl;
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
			intersection_to_save.add_sched_car(saving_car, target_car);
			Node_in_Car to_save_key(time_idx, intersection_id);
			target_car.records_intersection_in_database[&intersection_to_save] = { "scheduled", intersection_to_save.time_stamp };

			thread_affected_intersections.insert(pair(time_idx, &intersection_to_save));
			if (saving_car.id == "car_153")
				cout << " scheduled: " << intersection_to_save.time_stamp << " " << _NOW_SIMU_TIME + time_idx * _schedule_period << " " << time_idx << " " << intersection_to_save.id_str << " " << saving_car.id << " " << saving_car.D + saving_car.OT << " " << saving_car.OT << endl;
		}
	}

}

void testQ() {

}
