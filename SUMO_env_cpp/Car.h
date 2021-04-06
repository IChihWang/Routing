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
	double CC_shift = 0;
	double CC_shift_end = 0;

	string CC_state = "";
	double CC_slowdown_timer = 0;
		//self.CC_front_car = None
	bool CC_is_stop_n_go = false;


	Car() {}
	Car(string car_id, uint8_t length, uint8_t lane, char turn, char next_turn);

private:
	void set_turning(char turn, char next_turn);
};

class Car_Info {
public:
	IntersectionManager* intersection_manager_ptr;
};