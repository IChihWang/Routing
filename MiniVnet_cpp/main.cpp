#include "server.h"

using namespace std;

int _thread_num = 2;
string handle_request(string in_str);

int main(int argc, char const* argv[]) {

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
			cout << in_str << endl;
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

string handle_request(string in_str) {

	in_str.pop_back();	// Remove the ";" at the end
	stringstream ss_car(in_str);
	
	while (ss_car.good()) {
		string car_str;
		getline(ss_car, car_str, ';');
		stringstream ss_car_data(car_str);

		string car_id;
		getline(ss_car_data, car_id, ',');
		string car_state;
		getline(ss_car_data, car_state, ',');

		if (car_state.compare("EXIT") == 0) {
			// TODO: delete care from database
		}
		else if (car_state.compare("PAUSE") == 0) {
			// Cannot reroute the car due to the lower lever control
		}
		else if (car_state.compare("NEW") == 0 or car_state.compare("OLD") == 0) {
			string car_data;
			getline(ss_car_data, car_state, ',');
		}
		

	}

	string out_str;
	return out_str;
}
