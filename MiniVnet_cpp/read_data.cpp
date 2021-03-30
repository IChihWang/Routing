#include "global.h"

#include "json.hpp"
#include <fstream>
#include <utility>	//swap


using namespace std;
using json = nlohmann::json;


void read_load_adv_data() {
	ifstream file("./advise_info/advise_info" + to_string(LANE_NUM_PER_DIRECTION) + ".json");
	json data;
	file >> data;
	file.close();

	for (auto& elements : data.items()) {
		string key = elements.key();
		vector< map<char, uint8_t> > values;

		for (auto& accupied_boxes : elements.value()) {
			uint8_t x = accupied_boxes["X"];
			uint8_t y = accupied_boxes["Y"];

			map<char, uint8_t> tmp_data = { {'X', x}, {'Y', y} };
			values.push_back(tmp_data);
		}
		lane_dict[key] = values;
	}
}

void read_inter_info_data() {
	ifstream file("./inter_info/lane_info" + to_string(LANE_NUM_PER_DIRECTION) + ".json");
	json data;
	file >> data;
	file.close();

	for (auto& element : data.items()) {
		string key = element.key();

		double Ym = element.value()["Ym"];
		double Yd = element.value()["Yd"];
		double Xd = element.value()["Xd"];
		double Xm = element.value()["Xm"];

		map<string, double> tmp_data = { {"Ym", Ym}, {"Yd", Yd}, {"Xd", Xd}, {"Xm", Xm} };

		inter_info[key] = tmp_data;
	}
}


void read_inter_length_data() {
	ifstream file("./inter_length_info/lane_info" + to_string(LANE_NUM_PER_DIRECTION) + ".json");
	json data;
	file >> data;
	file.close();

	for (auto& element : data.items()) {
		string key = element.key();
		double trajectory_length = element.value();

		inter_length_dict[key] = trajectory_length;
	}
}

tuple<double, double> get_Conflict_Region(Car_in_database car1, Car_in_database car2) {
	bool reverse_flag = false;
	Car_in_database* car1_ptr = &car1;
	Car_in_database* car2_ptr = &car2;

	if (car1.lane > car2.lane) {
		swap(car1_ptr, car2_ptr);
		reverse_flag = true;
	}

	uint8_t l1 = car1_ptr->lane;
	uint8_t l2 = car2_ptr->lane;

	if (l1 >= LANE_NUM_PER_DIRECTION) {
		l2 = (l2 - (l1 / LANE_NUM_PER_DIRECTION) * LANE_NUM_PER_DIRECTION) % (LANE_NUM_PER_DIRECTION * 4);
		l1 = (l1 - (l1 / LANE_NUM_PER_DIRECTION) * LANE_NUM_PER_DIRECTION);
	}

	// Test if there is conflict between l1& l2 and give the conflict region.

	string test_str = to_string(l1) + car1_ptr->current_turn + to_string(l2) + car2_ptr->current_turn;
	if (inter_info.find(test_str) != inter_info.end()) {
		map<string, double>& val = inter_info[test_str];

		double val_Xm = val["Xm"];
		double val_Xd = val["Xd"];
		// Flip Xd, Yd if two car turn left from opposite directions
		if ((l2 >= LANE_NUM_PER_DIRECTION * 2 && l2 < LANE_NUM_PER_DIRECTION * 3) && (car1_ptr->current_turn == 'L' && car2_ptr->current_turn == 'L')) {
			swap(val_Xm, val_Xd);
		}

		// Compute tau_S1_S2
		double tau_S1_S2 = 0;
		// --- case 1: from Xm, Ym
		double ans1 = (val_Xm + car1_ptr->length + _HEADWAY) / car1_ptr->speed_in_intersection - (val["Ym"]) / car2_ptr->speed_in_intersection + DISTANCE * LANE_WIDTH / car1_ptr->speed_in_intersection;
		// --- case 2: from Xd, Yd
		double ans2 = (val_Xd + car1_ptr->length + _HEADWAY) / car1_ptr->speed_in_intersection - (val["Yd"]) / car2_ptr->speed_in_intersection + DISTANCE * LANE_WIDTH / car1_ptr->speed_in_intersection;
		tau_S1_S2 = max(ans1, ans2);

		// Compute tau_S2_S1
		double tau_S2_S1 = 0;
		// --- case 1: from Xm, Ym
		ans1 = (val["Ym"] + car2_ptr->length + _HEADWAY) / car2_ptr->speed_in_intersection - (val_Xm) / car1_ptr->speed_in_intersection + DISTANCE * LANE_WIDTH / car2_ptr->speed_in_intersection;
		// --- case 2: from Xd, Yd
		ans2 = (val["Yd"] + car2_ptr->length + _HEADWAY) / car2_ptr->speed_in_intersection - (val_Xd) / car1_ptr->speed_in_intersection + DISTANCE * LANE_WIDTH / car2_ptr->speed_in_intersection;
		tau_S2_S1 = max(ans1, ans2);

		if (reverse_flag) {
			return tuple<double, double>(tau_S2_S1, tau_S1_S2);	// swapped
		}
		else {
			return tuple<double, double>(tau_S1_S2, tau_S2_S1);
		}
	}
	else {
		return tuple<double, double>(0, 0);
	}
}

double get_Intertime(uint8_t lane, char turn) {
	string test_str = to_string(lane % LANE_NUM_PER_DIRECTION) + turn;
	double road_length = inter_length_dict[test_str];

	if (turn == 'S') {
		return road_length / _V_MAX;
	}
	else {
		return road_length / _TURN_SPEED;
	}
}