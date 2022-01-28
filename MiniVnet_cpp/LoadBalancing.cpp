#include "global.h"
#include <cmath>
using namespace std;

map< Coord, Coord> _intersection_MEC;
map< Edge_ID, Coord> _roadseg_MEC;
map< string, Coord>	_car_id_MEC_map;
vector<Coord> _MEC_id_list;
map< Coord, vector<Coord>> _MEC_intersection;	// Record the intersections that each MEC has
map<Coord, int> intersection_new_car_in;		// Record the number of new cars (new to the map) in each intersection
int _MEC_num_per_edge = 3;

void initial_district_allocation() {
	int _num_intersection_per_edge = ceil(1.0 * _grid_size / _MEC_num_per_edge);

	for (int i = 1; i <= _grid_size; i++) {
		for (int j = 1; j <= _grid_size; j++) {
			Coord MEC_id = Coord((i - 1) / _num_intersection_per_edge, (j - 1) / _num_intersection_per_edge);
			_MEC_id_list.push_back(MEC_id);
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
	//	Update with the _top_congested_intersections car number
	for (auto& [timestamp, intersection_ptr] : _top_congested_intersections) {
		Coord& intersection_id = intersection_ptr->id;
		Coord& MEC_id = _intersection_MEC[intersection_id];
		int car_num = intersection_ptr->get_car_num();
		MEC_car_num[MEC_id] += car_num;
		intersection_car_num[intersection_id] += car_num;
	}

	// Compute the load for each MEC
	map<Coord, double>	MEC_computation_load;	// Record the load in each MEC
	for (Coord& MEC_id : _MEC_id_list) {
		int V = get<0>(MEC_parameters[MEC_id]);
		int E = get<1>(MEC_parameters[MEC_id]);
		double computation_load = V + E * log2(V);	// Dijkstra's algorithm with min-priority queue
		MEC_computation_load[MEC_id] = computation_load;
	}

	// TODO: Starting from the highest load, try to offload to the neighbors
	// Sort the MEC by the load
	vector<Coord> sorted_MEC_id_list = _MEC_id_list;
	sort(sorted_MEC_id_list.begin(), sorted_MEC_id_list.end(),
		[&MEC_computation_load](const Coord& a, const Coord& b) -> bool
		{
			return MEC_computation_load[a] > MEC_computation_load[b];
		});

	// Repeat MEC_num time to propergate from high to low load
	for (int repeat_i = 0; repeat_i < _MEC_id_list.size(); repeat_i++) {
		// Get the one with the highest load
		Coord& highest_load_MEC_id = sorted_MEC_id_list[0];

		// Find the intersection with the most external to move
		vector<Coord> intersection_list = _MEC_intersection[highest_load_MEC_id];
		map<Coord, int> intersection_external_link_num;

		//	Calculate the external link number of each intersection
		for (const Coord& intersection_id : intersection_list) {
			int external_link_num = 0;
			for (int dir_i = 0; dir_i < 4; dir_i++) {
				Coord& external_MEC = _roadseg_MEC[Edge_ID(intersection_id, dir_i)];
				if (external_MEC != highest_load_MEC_id && external_MEC != OUTSIDE_MEC_MAP)
					external_link_num++;
			}
			if (external_link_num > 0)
				intersection_external_link_num[intersection_id] = external_link_num;
		}

		// Start negociate with the neighbers in the order of intersections with the most external link
		vector<Coord> searched_intersection;
		while (true) {
			map<Coord, int>::iterator chosen_intersection = max_element(intersection_external_link_num.begin(), intersection_external_link_num.end(),
				[](const pair<Coord, int> & a, const pair<Coord, int>& b) -> bool {
					return a.second < b.second;
				});
			const Coord& intersection_id = chosen_intersection->first;
			searched_intersection.push_back(intersection_id);

			// Calculate the potential E reduction on current MEC
			int potential_reduced_E = 0;
			for (int dir_i = 0; dir_i < 4; dir_i++) {
				Coord& connected_MEC = _roadseg_MEC[Edge_ID(intersection_id, dir_i)];
				if (connected_MEC == highest_load_MEC_id)
					potential_reduced_E++;
			}

			// Ask the neighbors
			for (int dir_i = 0; dir_i < 4; dir_i++) {
				Coord& neighbor_MEC = _roadseg_MEC[Edge_ID(intersection_id, dir_i)];
				// Well, the "neighbor_MEC" is itself in this case
				if (neighbor_MEC == highest_load_MEC_id)
					continue;
				// neighbor_MEC has higher or equal load than itself
				else if (MEC_computation_load[neighbor_MEC] >= MEC_computation_load[highest_load_MEC_id])
					continue;
				// Otherwise, found a potential neighbor to take over the load
				else {
					// Calculate the potential load reduction after moving the intersection
				}
			}

		}

		/*
		// Negociate with the neighbor and then update the MEC_computation_load
		// Also, update _intersection_MEC at the mean time
		// Find the neighbors
		int MEC_i = get<0>(highest_load_MEC_id);
		int MEC_j = get<1>(highest_load_MEC_id);
		vector<Coord> MEC_neighbors;
		if (MEC_i - 1 >= 0)
			MEC_neighbors.push_back(Coord(MEC_i - 1, MEC_j));
		if (MEC_j - 1 >= 0)
			MEC_neighbors.push_back(Coord(MEC_i, MEC_j - 1));
		if (MEC_i + 1 < _MEC_num_per_edge)
			MEC_neighbors.push_back(Coord(MEC_i + 1, MEC_j));
		if (MEC_j + 1 < _MEC_num_per_edge)
			MEC_neighbors.push_back(Coord(MEC_i, MEC_j + 1));
		*/



		// Remove the MEC with highest load, then re-sort the list
		sorted_MEC_id_list.erase(sorted_MEC_id_list.begin());
		sort(sorted_MEC_id_list.begin(), sorted_MEC_id_list.end(),
			[&MEC_computation_load](const Coord& a, const Coord& b) -> bool
			{
				return MEC_computation_load[a] > MEC_computation_load[b];
			});
	}


	// TODO: Finally update _intersection_MEC
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
	for (int i = 0; i <= _grid_size + 1; i++) {
		intersection_new_car_in[Coord(i, 0)] = 0;
		intersection_new_car_in[Coord(i, _grid_size + 1)] = 0;
	}
	for (int j = 0; j <= _grid_size + 1; j++) {
		intersection_new_car_in[Coord(0, j)] = 0;
		intersection_new_car_in[Coord(_grid_size + 1, j)] = 0;
	}

	// Update the load info
	for (string& car_id : new_car_ids) {
		Coord& intersection_id = _car_dict[car_id].src_coord;
		intersection_new_car_in[intersection_id]++;
	}
}