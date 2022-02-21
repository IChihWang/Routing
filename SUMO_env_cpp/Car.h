#pragma once
#include "global.h"
#include <string>
using namespace std;

class Car {
public:
	string id = "";
	uint8_t length = 0;
	char current_turn = '\0';
	char next_turn = '\0';

	uint8_t in_direction = 0;
	uint8_t out_direction = 0;
	uint8_t dst_lane = 0;
	uint8_t dst_lane_changed_to = 0;
	double speed_in_intersection = 0;

	// Information that might change during the simulation
	uint8_t original_lane = 0;	// The lane when the car joined the system
	uint8_t lane = 0;			// Current car lane
	uint8_t desired_lane = 0;	// The lane that the car wants to change
	bool is_spillback = false;
	bool is_spillback_strict = false;

	// Position: how far between it and the intersection (0 at the entry of intersection)
	double position = TOTAL_LEN;


	// Variables for Scheduling
	bool is_scheduled = false;
	double D = 0;
	double OT = 0;
	double Enter_T = 0;
	double Leave_T = 0;

	// Zone stage
	string zone = "";
	string zone_state = "";

	// Variables for Cruse Control
	double CC_front_pos_diff = 0;
	double CC_slow_speed = V_MAX;
	double CC_shift = -1;
	double CC_shift_end = CCZ_DEC2_LEN+2*CCZ_ACC_LEN;

	string CC_state = "";
	double CC_slowdown_timer = 0;
	Car* CC_front_car = nullptr;
	bool CC_is_stop_n_go = false;


	Car() {}
	Car(string car_id, uint8_t length, uint8_t lane, char turn, char next_turn);
	void handle_CC_behavior(map<string, Car*>& car_list);

	void set_turning(char turn, char next_turn);

private:
	double CC_get_front_speed();
	pair<double, double> CC_get_shifts(map<string, Car*>& car_list);	// return {'shifting': shifting, 'shifting_end': CC_shift_end}
	void CC_get_slow_down_speed();
};

class Car_Info {
public:
	IntersectionManager* intersection_manager_ptr = nullptr;
	uint8_t src_shift_num = 0;
	string dst_node_idx = "";
	string route_state = "";
	string inter_status = "";
	uint8_t car_length = 0;
	string route = "";
	bool is_last_turn = false;	// At the last turn, force turn
	
	double enter_time = 0;
	double shortest_travel_time = 0;
	double estimated_exit_time = 0;

	void compute_shortest_time(string in_lane_str, string dst_node_str) {
		int link_num = 1;
		link_num += abs(stoi(in_lane_str.substr(0, 3)) - stoi(dst_node_str.substr(0, 3)));
		link_num += abs(stoi(in_lane_str.substr(4, 3)) - stoi(dst_node_str.substr(4, 3)));
		
		int mid_road_length = 200;
		int outter_road_length = 500;
		shortest_travel_time = ((link_num - 2) * mid_road_length + outter_road_length * 2) / V_MAX;
	}
};