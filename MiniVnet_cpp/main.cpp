
#include "server.h"
#include "global.h"

#include <chrono>	// Measure runtime

using namespace std;
using namespace std::chrono;

uint8_t _thread_num;
map<string, map<string, double> > inter_info;
map<string, vector< map<char, uint8_t> >> lane_dict;
map<string, double> inter_length_dict;
uint8_t _ITERATION_NUM;
string _ARRIVAL_RATE;
string _RANDOM_SEED;
double _NOW_SIMU_TIME = 0;
int _CAR_TIME_ERROR = 0;
int _MEC_num_per_edge = 0;
string _Enable_Load_Balance;



ofstream route_result_file;
ofstream other_info_file;
ofstream load_balance_range_file;
ofstream all_MEC_computation_time_file;
ofstream all_MEC_car_number_file;
ofstream all_MEC_speed_file;
ofstream overall_speed_file;

double total_compuation_time = 0;
long total_routing_car_num = 0;
long total_new_car_num = 0;


int main(int argc, char const* argv[]) {

	read_load_adv_data();
	read_inter_info_data();
	read_inter_length_data();
	cout << "Done initialization, waiting for sumo..." << endl;

	SOCKET new_socket = initial_server_handler();

	// Initializations done after getting infos from SUMO
	create_grid_network();
	initial_avg_map();		// For car routing by themselves
	initial_district_allocation();
	init_thread_pool();

	// Create files for writing the results
	string file_name_prefix = string("result/") + to_string(_grid_size) + "_" +
		to_string(_TOP_N_CONGESTED) + "_" + to_string(_CHOOSE_CAR_OPTION) + "_" +
		to_string(_thread_num) + "_" + to_string(_ITERATION_NUM) + "_" +
		to_string(_CAR_TIME_ERROR) + "_" + _ARRIVAL_RATE + "_" + _RANDOM_SEED + "_" +
		to_string(_MEC_num_per_edge) + "_" + _Enable_Load_Balance + "_";

	route_result_file.open(file_name_prefix + "routes.csv");
	other_info_file.open(file_name_prefix + "other_info.csv");
	all_MEC_computation_time_file.open(file_name_prefix + "MEC_computation_time.csv");
	load_balance_range_file.open(file_name_prefix+"load_balance_range.csv");
	all_MEC_car_number_file.open(file_name_prefix + "MEC_car_num.csv");
	all_MEC_speed_file.open(file_name_prefix + "MEC_speed.csv");
	overall_speed_file.open(file_name_prefix + "overall_speed.csv");


	route_result_file << "car_id, path, estimated_travel_timie" << endl;
	other_info_file << "new_car_num, scheduling_car_num" << endl;
	overall_speed_file << "throughput (num/s)" << endl;
	for (const Coord& MEC_id : _MEC_id_list) {
		all_MEC_computation_time_file << MEC_id << "_computation_time,";
		load_balance_range_file << MEC_id << "_MEC_range,";
		all_MEC_car_number_file << MEC_id << "_MEC_car_num,";
		all_MEC_speed_file << MEC_id << "_MEC_speed,";
	}
	all_MEC_computation_time_file << endl;
	load_balance_range_file << endl;
	all_MEC_car_number_file << endl;
	all_MEC_speed_file << endl;

	// Receiving requests/sending replies
	while (true) {

		string in_str = "";
		bool is_connected = true;

		while (in_str.length() == 0 || in_str.back() != '@') {
			char buffer[1024] = { 0 };
			int n_recv = recv(new_socket, buffer, 1023, 0);
			if (n_recv <= 0) {
				is_connected = false;
				break;
			}
			in_str += buffer;
		}
		in_str.pop_back();
		if (!is_connected)
			break;


		string out_str = "";
		if (in_str.length() > 0) {
			out_str = handle_request(in_str);
		}

		out_str += '@';
		send(new_socket, out_str.c_str(), (int)out_str.length(), 0);

		if (out_str == "end") {
			break;
		}
	}

	route_result_file.close();
	other_info_file.close();
	load_balance_range_file.close();
	all_MEC_computation_time_file.close();
	all_MEC_car_number_file.close();
	all_MEC_speed_file.close();
	overall_speed_file.close();

	return 0;
}

string handle_request(string &in_str) {
	cout << "Get request" << endl;
	in_str.pop_back();	// Remove the ";" at the end
	stringstream ss_car(in_str);
	vector<string> new_car_ids;
	vector<string> old_car_ids;

	string simu_time_str;
	getline(ss_car, simu_time_str, ';');
	_NOW_SIMU_TIME = stod(simu_time_str);

	// Parse the data from SUMO
	while (ss_car.good()) {
		string car_str;
		getline(ss_car, car_str, ';');
		stringstream ss_car_data(car_str);

		string car_id;
		getline(ss_car_data, car_id, ',');
		string car_state;
		getline(ss_car_data, car_state, ',');

		if (car_state == "EXIT") {
			delete_car_from_database_id(car_id);
		}
		else if (car_state == "PAUSE") {
			// Cannot reroute the car due to the lower lever control
			if (_car_dict.find(car_id) != _car_dict.end())
				_car_dict[car_id].state = car_state;
		}
		else if (car_state == "NEW" || car_state == "OLD") {

			string car_data;
			getline(ss_car_data, car_data, ',');
			uint8_t car_length = stoi(car_data);
			getline(ss_car_data, car_data, ',');
			string src_intersection_id = car_data;
			getline(ss_car_data, car_data, ',');
			uint8_t direction_of_src_intersection = stoi(car_data);
			getline(ss_car_data, car_data, ',');
			uint16_t time_offset_step = stoi(car_data);
			getline(ss_car_data, car_data, ',');
			double position_at_offset = stod(car_data);
			getline(ss_car_data, car_data, ',');
			string dst_node_str = car_data;

			update_car(car_id, car_length, src_intersection_id,
				direction_of_src_intersection, time_offset_step,
				position_at_offset, dst_node_str);
			_car_dict[car_id].state = car_state;

			if (car_state == "NEW") {
				new_car_ids.push_back(car_id);
			}
			else if (car_state == "OLD") {
				old_car_ids.push_back(car_id);
			}
		}
	}

	// Record the new car number
	other_info_file << new_car_ids.size() << ',';
	total_new_car_num += new_car_ids.size();
	int count_scheduling_car_num = 0;

	// Record the result to tell SUMO
	map<string, string> routes_dict;

	map<string, double> traveling_time_dict;
	map<Coord, double> computation_time_list;
	for (Coord& MEC_id : _MEC_id_list) {
		computation_time_list[MEC_id] = 0;
	}

	if (_Enable_Load_Balance == "T") {
		// Load balancing
			// "Estimate" new car load (idea: pick the car num in last time window)
			// $ Measure the computation loads (Measurement or estimate mathmatically)
			// & Load balancing algorithm
			// & Move the intersections between districts (By modifying _intersection_MEC)
		load_balancing(new_car_ids);
	}
		// Classify groups for each MEC
		put_cars_into_districts();

// Cars route
	// Abstract the district info (Average on each road)
	compute_avg_map();
	// Route for every car
	brief_route();
	// Acclocate temporary destination for cars within a district
	decide_tmp_destination();


	// Global varibale for next round, local for each district
	vector<pair<int32_t, Intersection*>> top_congested_intersections = _top_congested_intersections;
	_top_congested_intersections.clear();

	// Run routing in each district
	for (Coord& MEC_id: _MEC_id_list){

		// Record top N congested list for a district
		vector< pair<int32_t, Intersection*> > district_top_congested_intersections = top_congested_intersections;

		auto begin = high_resolution_clock::now();
		for (int iter_i = 0; iter_i < _ITERATION_NUM; iter_i++) {
			vector<vector<reference_wrapper<Car>>> route_groups;
			route_groups = choose_car_to_thread_group(MEC_id, new_car_ids, district_top_congested_intersections);

			//Remove the new cars after first routing
			new_car_ids.erase(
				std::remove_if(
					new_car_ids.begin(),
					new_car_ids.end(),
					[&](string const& p) { return _car_id_MEC_map[p] == MEC_id; }
				),
				new_car_ids.end()
			);


			// routing_with_groups(route_groups, routes_dict);
			bool is_no_routing = true;
			for (auto route_group : route_groups) {
				if (route_group.size() > 0)
					is_no_routing = false;
			}
			if (!is_no_routing) {
				routing_with_groups_thread(MEC_id, route_groups, routes_dict);

				// Find the next top N congested list
				add_intersection_to_reschedule_list(district_top_congested_intersections);
			}
			else {
				district_top_congested_intersections.clear();
			}

			// Updated during routing, so no need to update database here
			// router.update_database_after_routing(route_groups)

			// Get the traveling time of cars (for statistic)
			for (uint8_t i = 0; i < _thread_num; i++) {
				for (Car& car : route_groups[i]) {
					traveling_time_dict[car.id] = car.traveling_time;
					count_scheduling_car_num++;
				}
			}
		}

		// Record the top N intersection globally
		_top_congested_intersections.insert(_top_congested_intersections.end(), district_top_congested_intersections.begin(), district_top_congested_intersections.end());

		auto end = high_resolution_clock::now();
		auto route_time = duration<double>(end - begin);
		cout << "District " << get<0>(MEC_id) << "," << get<1>(MEC_id) << " : Route_time: " << route_time.count() << " seconds" << endl;
		computation_time_list[MEC_id] = route_time.count();
	}

	// Finalize the results
	string out_str = "";
	for (const pair<string, string>& route_data : routes_dict) {
		string car_id = route_data.first;
		string path = route_data.second;
		out_str += car_id;
		out_str += ',';
		out_str += path;
		out_str += ',';
		out_str += to_string(traveling_time_dict[car_id]);
		out_str += ';';
		route_result_file << car_id << ',' << path << traveling_time_dict[car_id] << endl;
	}

	other_info_file << count_scheduling_car_num;
	other_info_file << endl;

	// Write the result files
	for (Coord& MEC_id : _MEC_id_list) {
		all_MEC_computation_time_file << computation_time_list[MEC_id] << ",";
	}
	all_MEC_computation_time_file << endl;

	for (Coord& MEC_id : _MEC_id_list) {
		for (Coord& intersection_id : _MEC_intersection[MEC_id]) {
			load_balance_range_file << intersection_id << "|";
		}
		load_balance_range_file << ",";
	}
	load_balance_range_file << endl;

	for (Coord& MEC_id : _MEC_id_list) {
		all_MEC_car_number_file << _MEC_car_num[MEC_id] << ",";
	}
	all_MEC_car_number_file << endl;

	for (Coord& MEC_id : _MEC_id_list) {
		all_MEC_speed_file << _MEC_car_num[MEC_id]/computation_time_list[MEC_id] << ",";
	}
	all_MEC_speed_file << endl;

	int total_car_num = 0;
	double max_computation_time = 0;
	for (Coord& MEC_id : _MEC_id_list) {
		total_car_num += _MEC_car_num[MEC_id];
		max_computation_time = max(max_computation_time, computation_time_list[MEC_id]);
	}
	overall_speed_file << total_car_num / max_computation_time << endl;


	move_a_time_step();

	return out_str;
}

ostream& operator<<(ostream& os, const Coord& coord_id)
{
	os << "(" << get<0>(coord_id) << '_' << get<1>(coord_id) << ')';
	return os;
}
