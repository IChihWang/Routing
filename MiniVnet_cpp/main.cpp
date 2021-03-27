#include "server.h"
#include "global.h"



uint8_t _thread_num = 2;
map<string, map<string, double> > inter_info;
map<string, vector< map<char, uint8_t> >> lane_dict;
map<string, double> inter_length_dict;


int main(int argc, char const* argv[]) {
	testQ();
	read_load_adv_data();
	read_inter_info_data();
	read_inter_length_data();

	int new_socket = initial_server_handler();
	create_grid_network();


	// Receiving requests/sending replies
	while (true) {

		string in_str = "";

		while (in_str.length() == 0 or in_str.back() != '@') {
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
			// TODO: routing
			out_str = handle_request(in_str);
		}

		out_str += '@';
		send(new_socket, out_str.c_str(), out_str.length(), 0);

		if (out_str == "end") {
			return 0;
		}
	}

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
		cout << car_str << endl;

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
		else if (car_state.compare("NEW") == 0 or car_state.compare("OLD") == 0) {
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

		routing_with_groups(route_groups, routes_dict);
	}


	string out_str;
	return out_str;
}
