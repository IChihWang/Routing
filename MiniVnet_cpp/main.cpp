
#include "server.h"
#include "global.h"

#include <chrono>	// Measure runtime

#include <fstream>
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



ofstream route_result_file;
ofstream all_computation_time_file;
ofstream statistic_file;

int routing_count = 0;
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
	init_thread_pool();

	// Create files for writing the results
	string file_name_prefix = string("result/") + to_string(_grid_size) + "_" +
		to_string(_TOP_N_CONGESTED) + "_" + to_string(_CHOOSE_CAR_OPTION) + "_" +
		to_string(_thread_num) + "_" + to_string(_ITERATION_NUM) + "_" + to_string(_CAR_TIME_ERROR) + "_" + _ARRIVAL_RATE + "_" + _RANDOM_SEED + "_";
	route_result_file.open(file_name_prefix + "routes.csv");
	all_computation_time_file.open(file_name_prefix + "computation_time.csv");
	statistic_file.open("result/statistic.csv", ofstream::app);

	route_result_file << "car_id, path, estimated_travel_timie" << endl;
	all_computation_time_file << "new_car_num, all_car_num, compuation_time" << endl;

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

	statistic_file << "Grid size, top N, choose_car, thread_num, iteration_num, _CAR_TIME_ERROR, arrival_rate, rand_seed, avg_compuation_time, avg_route_num, avg_new_car_num, total_new_car_num, compuation_time_per_car, compuation_time_per_new_car, routing_count" << endl;
	statistic_file << (int)_grid_size << ',' << (int)_TOP_N_CONGESTED << ',' << (int)_CHOOSE_CAR_OPTION << ',' << (int)_thread_num << ',' << (int)_ITERATION_NUM << ',' << _CAR_TIME_ERROR << ',';
	statistic_file << _ARRIVAL_RATE << ',' << _RANDOM_SEED << ',' << total_compuation_time / routing_count << ','; 
	statistic_file << total_routing_car_num / routing_count << ',' << total_new_car_num / routing_count << ',';
	statistic_file << total_new_car_num << ',' << total_compuation_time/ total_routing_car_num << ',' << total_compuation_time / total_new_car_num << ',' << routing_count << endl;
	statistic_file.close();

	route_result_file.close();
	all_computation_time_file.close();



	//terminate_thread_pool();

	return 0;
}

string handle_request(string &in_str) {
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

		if (car_state.compare("EXIT") == 0) {
			delete_car_from_database_id(car_id);
		}
		else if (car_state.compare("PAUSE") == 0) {
			// Cannot reroute the car due to the lower lever control
			if (_car_dict.find(car_id) != _car_dict.end())
				_car_dict[car_id].state = car_state;
		}
		else if (car_state.compare("NEW") == 0 || car_state.compare("OLD") == 0) {
			
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

			if (car_state.compare("NEW") == 0) {
				new_car_ids.push_back(car_id);
			}
			else if (car_state.compare("OLD") == 0) {
				old_car_ids.push_back(car_id);
			}
		}
	}

	// Record the new car number
	all_computation_time_file << new_car_ids.size() << ',';
	total_new_car_num += new_car_ids.size();
	int count_scheduling_car_num = 0;

	map<string, string> routes_dict;
	map<string, double> traveling_time_dict;
	vector<double> computation_time_list;

// TODO
// Load balancing
	// Measure the computation loads (Measurement or estimate mathmatically)
	// Load balancing algorithm
	// Move the intersections between districts
// Cars route
	// Abstract the district info (Average on each road)
	compute_avg_map();
	// Route for every car
	brief_route();

// TODO: cut the city into districts




	auto begin = high_resolution_clock::now();
	for (int iter_i = 0; iter_i < _ITERATION_NUM; iter_i++) {
		vector<vector<reference_wrapper<Car>>> route_groups;
		route_groups = choose_car_to_thread_group(new_car_ids, old_car_ids);

		new_car_ids.clear();	//Remove the new cars after first routing

		// Clear affected_intersection_list before routing starts 
		affected_intersections.clear();

		// routing_with_groups(route_groups, routes_dict);
		bool is_no_routing = true;
		for (auto route_group : route_groups) {
			if (route_group.size() > 0)
				is_no_routing = false;
		}
		if (!is_no_routing)
			routing_with_groups_thread(route_groups, routes_dict);
		
		// Find the next top N congested list
		add_intersection_to_reschedule_list();

		// Updated during routing, so no need to update database here
		// router.update_database_after_routing(route_groups)

		// Get the traveling time of cars
		for (uint8_t i = 0; i < _thread_num; i++) {
			for (Car& car : route_groups[i]) {
				traveling_time_dict[car.id] = car.traveling_time;
				count_scheduling_car_num++;
			}
		}
	}
	auto end = high_resolution_clock::now();

	auto route_time = duration<double>(end - begin);
	cout << "Route_time: " << route_time.count() << " seconds" << endl;
	computation_time_list.push_back(route_time.count());


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
		route_result_file << car_id << ',' << path << endl;
	}

	all_computation_time_file << count_scheduling_car_num << ',' << route_time.count() << endl;
	routing_count++;
	total_compuation_time += route_time.count();
	total_routing_car_num += count_scheduling_car_num;

	move_a_time_step();

	return out_str;
}

