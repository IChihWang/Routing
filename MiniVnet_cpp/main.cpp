
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



ofstream route_result_file;


int main(int argc, char const* argv[]) {

	route_result_file.open("result/route_result.csv");
	read_load_adv_data();
	read_inter_info_data();
	read_inter_length_data();
	cout << "Done initialization, waiting for sumo..." << endl;

	SOCKET new_socket = initial_server_handler();

	// Initializations done after getting infos from SUMO
	create_grid_network();
	init_thread_pool();

	// Receiving requests/sending replies
	while (true) {

		string in_str = "";

		while (in_str.length() == 0 || in_str.back() != '@') {
			char buffer[1024] = { 0 };
			int n_recv = recv(new_socket, buffer, 1023, 0);
			if (n_recv <= 0) {
				return 0;
			}
			in_str += buffer;
		}
		in_str.pop_back();


		string out_str = "";
		if (in_str.length() > 0) {
			out_str = handle_request(in_str);
		}

		out_str += '@';
		send(new_socket, out_str.c_str(), (int)out_str.length(), 0);

		if (out_str == "end") {
			return 0;
		}
	}

	terminate_thread_pool();

	return 0;
}

string handle_request(string &in_str) {
	in_str.pop_back();	// Remove the ";" at the end
	stringstream ss_car(in_str);
	vector<string> new_car_ids;
	vector<string> old_car_ids;
	
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

			if (car_state.compare("NEW") == 0) {
				new_car_ids.push_back(car_id);
			}
			else if (car_state.compare("OLD") == 0) {
				old_car_ids.push_back(car_id);
			}
		}
	}

	map<string, string> routes_dict;

	for (int iter_i = 0; iter_i < 1; iter_i++) {
		vector<vector<reference_wrapper<Car>>> route_groups;
		route_groups = choose_car_to_thread_group(new_car_ids, old_car_ids);

		// Clear affected_intersection_list before routing starts 
		affected_intersections.clear();

		auto begin = high_resolution_clock::now();
		// routing_with_groups(route_groups, routes_dict);
		routing_with_groups_thread(route_groups, routes_dict);
		auto end = high_resolution_clock::now();

		auto route_time = duration<double>(end - begin);
		cout << "Route_time: " << route_time.count() << " seconds" << endl;

		// Find the next top N congested list
		add_intersection_to_reschedule_list();

		// Updated during routing, so no need to update database here
		// router.update_database_after_routing(route_groups)
	}


	// Finalize the results
	string out_str = "";
	for (const pair<string, string>& route_data : routes_dict) {
		string car_id = route_data.first;
		string path = route_data.second;
		out_str += car_id;
		out_str += ',';
		out_str += path;
		out_str += ';';
		route_result_file << car_id << ',' << path << endl;
	}


	move_a_time_step();

	return out_str;
}

