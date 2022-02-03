#include "global.h"
#include <cmath>
using namespace std;

map< Coord, Coord> _intersection_MEC;
map< Edge_ID, Coord> _roadseg_MEC;
map< string, Coord>	_car_id_MEC_map;
vector<Coord> _MEC_id_list;
map< Coord, vector<Coord>> _MEC_intersection;	// Record the intersections that each MEC has
map< Coord, int> intersection_new_car_in;		// Record the number of new cars (new to the map) in each intersection
int _MEC_num_per_edge = 3;
map< Coord, Coord> MEC_center_coord;			// Record the center of the MEC to prevent overly migration

void initial_district_allocation() {
	int _num_intersection_per_edge = ceil(1.0 * _grid_size / _MEC_num_per_edge);

	// Initial and allocate intersections to MECs
	for (int i = 1; i <= _grid_size; i++) {
		for (int j = 1; j <= _grid_size; j++) {
			Coord MEC_id = Coord((i - 1) / _num_intersection_per_edge, (j - 1) / _num_intersection_per_edge);
			if (find(_MEC_id_list.begin(), _MEC_id_list.end(), MEC_id) == _MEC_id_list.end()) {
				_MEC_id_list.push_back(MEC_id);
			}
			_intersection_MEC[Coord(i, j)] = MEC_id;
		}
	}

	// Index the edge intersection of the whole city map  (OUTSIDE_MEC_MAP)
	for (int i = 0; i <= _grid_size+1; i++) {
		_intersection_MEC[Coord(i, 0)] = OUTSIDE_MEC_MAP;
		_intersection_MEC[Coord(i, _grid_size + 1)] = OUTSIDE_MEC_MAP;
	}
	for (int j = 0; j <= _grid_size+1; j++) {
		_intersection_MEC[Coord(0, j)] = OUTSIDE_MEC_MAP;
		_intersection_MEC[Coord(_grid_size + 1, j)] = OUTSIDE_MEC_MAP;
	}

	// Decide the center intersection of each MEC (MEC_center_coord)
	for (const Coord& MEC_id : _MEC_id_list) {
		const int &MEC_i = get<0>(MEC_id);
		const int &MEC_j = get<1>(MEC_id);
		int center_i = MEC_i * _num_intersection_per_edge + _num_intersection_per_edge / 2 + 1;
		int center_j = MEC_j * _num_intersection_per_edge + _num_intersection_per_edge / 2 + 1;
		MEC_center_coord[MEC_id] = Coord(center_i, center_j);
	}

}

void put_cars_into_districts(){
	for (auto& [car_id, car] : _car_dict) {
		Coord& MEC_id = _intersection_MEC[car.src_coord];
		_car_id_MEC_map[car_id] = MEC_id;
	}
}

void load_balancing() {	// update _MEC_id_computation_load
	// Properly allocate the MEC_id for each intersection and road segment
	update_intersection_info_in_MEC();

	// Calculate the parameters in each MEC
	map<Coord, pair<int, int>> MEC_parameters;	// Record current |V| and |E| for each MEC
	for (Coord& MEC_id : _MEC_id_list) {
		MEC_parameters[MEC_id] = calculate_MEC_cost_info(MEC_id);
	}

	// Get the car numbers for each MEC and each intersection (for further load balancing)
	map<Coord, int>	MEC_car_num;	// Record the car number in each MEC
	map<Coord, int> intersection_car_num;
	//	Clear the number
	for (Coord& MEC_id : _MEC_id_list) {
		MEC_car_num[MEC_id] = 0;
	}
	for (int i = 1; i <= _grid_size; i++) {
		for (int j = 1; j <= _grid_size; j++) {
			intersection_car_num[Coord(i, j)] = 0;
		}
	}
	//	Update with the new car number
	for (auto& [intersection_id, new_car_num] : intersection_new_car_in) {
		Coord& MEC_id = _intersection_MEC[intersection_id];
		MEC_car_num[MEC_id] += new_car_num;
		intersection_car_num[intersection_id] += new_car_num;
	}

	map<Coord, int> debug_MEC_car_num;
	for (Coord& MEC_id : _MEC_id_list) {
		debug_MEC_car_num[MEC_id] = MEC_car_num[MEC_id];
	}
	

	//	Update with the _top_congested_intersections car number
	set<string> top_intersection_car_list;		// Record this in case an intersection is chosen twice or more with two different timestamp
	//		Gather all the car_to_be_updated
	for (auto& [timestamp, intersection_ptr] : _top_congested_intersections) {
		Coord& intersection_id = intersection_ptr->id;
		for (auto car_item : *(intersection_ptr->scheduling_cars)) {
			const string& car_id = car_item.first;
			if (_car_dict[car_id].state == "OLD" || _car_dict[car_id].state == "NEW") {
				top_intersection_car_list.insert(car_id);
			}
		}
	}
	//		Update the car number
	for (auto& car_id : top_intersection_car_list) {
		const Coord& intersection_id = _car_dict[car_id].src_coord;
		Coord& MEC_id = _intersection_MEC[intersection_id];
		MEC_car_num[MEC_id] += 1* _ITERATION_NUM;
		intersection_car_num[intersection_id] += 1* _ITERATION_NUM;
		debug_MEC_car_num[MEC_id]++;
	}

	

	// Compute the load for each MEC
	map<Coord, double>	MEC_computation_load;	// Record the load in each MEC
	for (Coord& MEC_id : _MEC_id_list) {
		int V = get<0>(MEC_parameters[MEC_id]);
		int E = get<1>(MEC_parameters[MEC_id]);
		double computation_load = V + E * log2(V);	// Dijkstra's algorithm with min-priority queue
		computation_load *= MEC_car_num[MEC_id];
		MEC_computation_load[MEC_id] = computation_load;
	}

	
	// Repeat MEC_num time to propergate from high to low load
	vector<Coord> candidate_MEC_id_list = _MEC_id_list;
	for (int repeat_i = 0; repeat_i < _MEC_id_list.size(); repeat_i++) {
		// Get the one with the highest load
		vector<Coord>::iterator chosen_MEC = max_element(candidate_MEC_id_list.begin(), candidate_MEC_id_list.end(),
			[&MEC_computation_load](const Coord& a, const Coord& b) -> bool {
				return MEC_computation_load[a] > MEC_computation_load[b];
			});
		Coord& highest_load_MEC_id = *(chosen_MEC);

		// Find the intersection with the most external to move
		vector<Coord>& intersection_list = _MEC_intersection[highest_load_MEC_id];
		map<Coord, int> candidate_intersection_external_link_num;

		//	Calculate the external link number of each intersection
		for (const Coord& intersection_id : intersection_list) {
			int external_link_num = 0;
			for (int dir_i = 0; dir_i < 4; dir_i++) {
				Coord& external_MEC = _roadseg_MEC[Edge_ID(intersection_id, dir_i)];
				if (external_MEC != highest_load_MEC_id && external_MEC != OUTSIDE_MEC_MAP)
					external_link_num++;
			}
			candidate_intersection_external_link_num[intersection_id] = external_link_num;
		}

		// Start negociate with the neighbers in the order of intersections with the most external link
		int candidate_intersection_num = candidate_intersection_external_link_num.size();	// Specify this because the size of the candidate_intersection changes within the loop
		for(int search_count = 0; search_count < candidate_intersection_num; search_count++){
			// Choose the intersection and record it
			map<Coord, int>::iterator chosen_intersection = max_element(candidate_intersection_external_link_num.begin(), candidate_intersection_external_link_num.end(),
				[](const pair<Coord, int> & a, const pair<Coord, int>& b) -> bool {
					return a.second < b.second;
				});
			const Coord& intersection_id = chosen_intersection->first;

			// Early terminate when there is no intersection to be searched (searched ones are removed)
			if (candidate_intersection_external_link_num[intersection_id] == 0)
				break;

			// Calculate the potential E reduction on current MEC
			int potential_reduced_E = 0;
			for (int dir_i = 0; dir_i < 4; dir_i++) {
				Coord& connected_MEC = _roadseg_MEC[Edge_ID(intersection_id, dir_i)];
				if (connected_MEC == highest_load_MEC_id)
					potential_reduced_E++;
			}

			// Calculate the potential moved car number on current MEC
			int car_number_change = 0;
			if (intersection_car_num.find(intersection_id) != intersection_car_num.end())
				car_number_change = intersection_car_num[intersection_id];

			// Ask the neighbors
			for (int dir_i = 0; dir_i < 4; dir_i++) {
				// Check whether the intersection is allowed to migrate in this direction (beyond the center; if so, skip)
				if (dir_i == 0) {
					if (get<1>(intersection_id) >= get<1>(MEC_center_coord[highest_load_MEC_id]))
						continue;
				}
				else if (dir_i == 1) {
					if (get<0>(intersection_id) <= get<0>(MEC_center_coord[highest_load_MEC_id]))
						continue;
				}
				else if (dir_i == 2) {
					if (get<1>(intersection_id) <= get<1>(MEC_center_coord[highest_load_MEC_id]))
						continue;
				}
				else if (dir_i == 3) {
					if (get<0>(intersection_id) >= get<0>(MEC_center_coord[highest_load_MEC_id]))
						continue;
				}


				Coord& neighbor_MEC = _roadseg_MEC[Edge_ID(intersection_id, dir_i)];

				if (highest_load_MEC_id == Coord(0, 0))
					cout << "The (0,0) has " << intersection_id << " , " << dir_i << " neighbor " << neighbor_MEC << " center " << MEC_center_coord[highest_load_MEC_id] << endl;

				// Well, the "neighbor_MEC" is itself in this case or outside city range
				if (neighbor_MEC == highest_load_MEC_id || neighbor_MEC == OUTSIDE_MEC_MAP)
					continue;
				// neighbor_MEC has higher or equal load than itself
				else if (MEC_computation_load[neighbor_MEC] >= MEC_computation_load[highest_load_MEC_id])
					continue;
				// Otherwise, found a potential neighbor to take over the load
				else {
					// Calculate the potential load reduction after moving the intersection
					int new_V = get<0>(MEC_parameters[highest_load_MEC_id]) -1;
					int new_E = get<1>(MEC_parameters[highest_load_MEC_id]) - potential_reduced_E;
					int new_car_num = MEC_car_num[highest_load_MEC_id] - car_number_change;

					double new_computation_load = new_V + new_E * log2(new_V);	// Dijkstra's algorithm with min-priority queue
					new_computation_load *= new_car_num;

					// Calculate the potential load gain of the neighbor
					int new_neighbor_V = get<0>(MEC_parameters[neighbor_MEC]) + 1;
					int neighbor_potential_gained_E = 0;
					for (int dir_i = 0; dir_i < 4; dir_i++) {
						Coord& connected_MEC = _roadseg_MEC[Edge_ID(intersection_id, dir_i)];
						if (connected_MEC == neighbor_MEC)
							neighbor_potential_gained_E++;
					}
					int new_neighbor_E = get<1>(MEC_parameters[neighbor_MEC]) + neighbor_potential_gained_E;
					int new_neighbor_car_num = MEC_car_num[neighbor_MEC] + car_number_change;
					double new_neighbor_computation_load = new_neighbor_V + new_neighbor_E * log2(new_neighbor_V);	// Dijkstra's algorithm with min-priority queue
					new_neighbor_computation_load *= new_neighbor_car_num;

					// Offload to the neighbor if the migration doesn't cost overload to the neighbor
					if (new_neighbor_computation_load < new_computation_load) {
						// Move the intersection
						_intersection_MEC[intersection_id] = neighbor_MEC;

						// Update the road segment info
						const int& intersection_id_i = get<0>(intersection_id);
						const int& intersection_id_j = get<1>(intersection_id);
						Coord neighbor_intersection_id(intersection_id_i, intersection_id_j - 1);
						if (_intersection_MEC[neighbor_intersection_id] != OUTSIDE_MEC_MAP) {
							_roadseg_MEC[Edge_ID(neighbor_intersection_id, 2)] = neighbor_MEC;
							// Update the neighbor in list candidate_intersection_external_link_num if the neighbor is a candidate (because the internal link changes to external)
							if (candidate_intersection_external_link_num.find(neighbor_intersection_id) != candidate_intersection_external_link_num.end()) {
								candidate_intersection_external_link_num[neighbor_intersection_id]++;
							}
						}
						neighbor_intersection_id = Coord(intersection_id_i + 1, intersection_id_j);
						if (_intersection_MEC[neighbor_intersection_id] != OUTSIDE_MEC_MAP) {
							_roadseg_MEC[Edge_ID(neighbor_intersection_id, 3)] = neighbor_MEC;
							// Update the neighbor in list candidate_intersection_external_link_num if the neighbor is a candidate (because the internal link changes to external)
							if (candidate_intersection_external_link_num.find(neighbor_intersection_id) != candidate_intersection_external_link_num.end()) {
								candidate_intersection_external_link_num[neighbor_intersection_id]++;
							}
						}
						neighbor_intersection_id = Coord(intersection_id_i, intersection_id_j + 1);
						if (_intersection_MEC[neighbor_intersection_id] != OUTSIDE_MEC_MAP) {
							_roadseg_MEC[Edge_ID(neighbor_intersection_id, 0)] = neighbor_MEC;
							// Update the neighbor in list candidate_intersection_external_link_num if the neighbor is a candidate (because the internal link changes to external)
							if (candidate_intersection_external_link_num.find(neighbor_intersection_id) != candidate_intersection_external_link_num.end()) {
								candidate_intersection_external_link_num[neighbor_intersection_id]++;
							}
						}
						neighbor_intersection_id = Coord(intersection_id_i - 1, intersection_id_j);
						if (_intersection_MEC[neighbor_intersection_id] != OUTSIDE_MEC_MAP) {
							_roadseg_MEC[Edge_ID(neighbor_intersection_id, 1)] = neighbor_MEC;
							// Update the neighbor in list candidate_intersection_external_link_num if the neighbor is a candidate (because the internal link changes to external)
							if (candidate_intersection_external_link_num.find(neighbor_intersection_id) != candidate_intersection_external_link_num.end()) {
								candidate_intersection_external_link_num[neighbor_intersection_id]++;
							}
						}

						// Update the MEC intersection
						_MEC_intersection[highest_load_MEC_id].erase(
							remove(_MEC_intersection[highest_load_MEC_id].begin(), _MEC_intersection[highest_load_MEC_id].end(), intersection_id)
							, _MEC_intersection[highest_load_MEC_id].end());
						_MEC_intersection[neighbor_MEC].push_back(intersection_id);
						
						// Update MEC parameters
						MEC_parameters[highest_load_MEC_id] = make_pair(new_V, new_E);
						MEC_parameters[neighbor_MEC] = make_pair(new_neighbor_V, new_neighbor_E);

						MEC_car_num[highest_load_MEC_id] = new_car_num;
						MEC_car_num[neighbor_MEC] = new_neighbor_car_num;

						int car_number_change2 = 0;
						if (intersection_car_num.find(intersection_id) != intersection_car_num.end())
							car_number_change2 = intersection_car_num[intersection_id];
						debug_MEC_car_num[highest_load_MEC_id] -= car_number_change2;
						debug_MEC_car_num[neighbor_MEC] += car_number_change2;

						// Update the load of the MEC
						MEC_computation_load[highest_load_MEC_id] = new_computation_load;
						MEC_computation_load[neighbor_MEC] = new_neighbor_computation_load;

						break;
					}
				}
			}

			// After asking the neighbors, remove the intersection out of the list to search
			candidate_intersection_external_link_num.erase(intersection_id);

		}

		// Remove the MEC with highest load
		candidate_MEC_id_list.erase(
			remove(candidate_MEC_id_list.begin(), candidate_MEC_id_list.end(), highest_load_MEC_id)
			, candidate_MEC_id_list.end());
	}

	for (Coord& MEC_id : _MEC_id_list) {
		_debug_file << debug_MEC_car_num[MEC_id] << ",";
	}
	for (Coord& MEC_id : _MEC_id_list) {
		_debug_file << MEC_car_num[MEC_id] << ",";
	}
	_debug_file << endl;

	// _intersection_MEC is updated in the code
}

pair<int, int> calculate_MEC_cost_info(Coord& MEC_id) {
	// Retrieve the intersection list of that MEC
	vector<Coord>& intersection_list = _MEC_intersection[MEC_id];

	// Decide the |V| in the district
	int intersection_num = (int)intersection_list.size();

	// Decide the |E| in the district
	int road_segment_num = 0;
	//	Check whether each road segment belongs to the current MEC
	for (Coord intersection_id : intersection_list) {
		int& i = get<0>(intersection_id);
		int& j = get<1>(intersection_id);

		// Check the four connections of this intesection
		for (int dir_i = 0; dir_i < 4; dir_i++) {
			if (_roadseg_MEC[Edge_ID(intersection_id, dir_i)] == MEC_id) {
				road_segment_num++;
			}
		}
	}

	return make_pair(intersection_num, road_segment_num);
}

void update_intersection_info_in_MEC() {
	// Clear the list for every MEC
	for (Coord& MEC_id : _MEC_id_list) {
		_MEC_intersection[MEC_id] = vector<Coord>();
	}

	// update the intersection list and road_segment list
	for (int i = 1; i <= _grid_size; i++) {
		for (int j = 1; j <= _grid_size; j++) {
			Coord intersection_id = Coord(i, j);
			Coord& MEC_id = _intersection_MEC[intersection_id];
			_MEC_intersection[MEC_id].push_back(intersection_id);

			// Update the MEC id for each road_segment
			_roadseg_MEC[Edge_ID(intersection_id, 0)] = _intersection_MEC[Coord(i, j-1)];
			_roadseg_MEC[Edge_ID(intersection_id, 1)] = _intersection_MEC[Coord(i+1, j)];
			_roadseg_MEC[Edge_ID(intersection_id, 2)] = _intersection_MEC[Coord(i, j+1)];
			_roadseg_MEC[Edge_ID(intersection_id, 3)] = _intersection_MEC[Coord(i-1, j)];
		}
	}
}

void estimate_new_car_load(vector<string> new_car_ids) {
	// Clear the load info
	intersection_new_car_in.clear();

	// Update the load info
	for (string& car_id : new_car_ids) {
		Coord& intersection_id = _car_dict[car_id].src_coord;
		if (intersection_new_car_in.find(intersection_id) == intersection_new_car_in.end())
			intersection_new_car_in[intersection_id] == 0;

		intersection_new_car_in[intersection_id]++;
	}

}