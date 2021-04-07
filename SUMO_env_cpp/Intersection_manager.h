#pragma once
#include <tuple>
#include <map>
#include "Car.h"
#include "LaneAdviser.h"
using namespace std;

#ifndef COORD
#define COORD
typedef tuple<uint16_t, uint16_t> Coord;
#endif

class Car_Delay_Position_Record {
public:
	double position = 0;
	double delay = 0;

	Car_Delay_Position_Record() {}
	Car_Delay_Position_Record(double in_position, double in_delay) : position(in_position), delay(in_delay) {}
};
class Road_Info {
public:
	uint16_t avail_len = TOTAL_LEN;
	vector<Car_Delay_Position_Record> car_delay_position;
};
class Car_Info_In_Intersection {
public:
	bool is_skip_car = false;
	double position_at_offset = 0;
	uint16_t time_offset_step = 0;
	string src_intersection_id = "";
	uint8_t direction_of_src_intersection = 0;
	uint8_t src_shift_num = 0;

	Car_Info_In_Intersection(){}
	Car_Info_In_Intersection(bool is_skip_car):is_skip_car(is_skip_car){}
	Car_Info_In_Intersection(double position_at_offset, uint16_t time_offset_step, 
		string src_intersection_id, uint8_t direction_of_src_intersection, uint8_t src_shift_num) :
		is_skip_car(false), position_at_offset(position_at_offset), time_offset_step(time_offset_step), 
		src_intersection_id(src_intersection_id), direction_of_src_intersection(direction_of_src_intersection), src_shift_num(src_shift_num) {}
};

class IntersectionManager {
public:
	Coord id;
	string id_str = "";

	// Break down to these to reduce the search space
	map<string, Car*> car_list;		// Cars that needs to be handled
	map<string, Car*> az_list;
	map<string, Car*> pz_list;
	map<string, Car*> cc_list;		// Cars under Cruse Control in CCZ
	map<string, Car*> leaving_cars;	// Cars just entered the intersection (leave the CC zone)

	double schedule_period_count = 0;

	Lane_Adviser lane_advisor;
	//scheduling_thread = None
	vector<string> in_lanes;
	vector<string> out_lanes;

	// For front car
	map<uint8_t, Car*> CC_last_cars_on_lanes;

	// Pedestrian control
	bool is_pedestrian_list[4] = {};				// Whether there is a pedestrian request
	double pedestrian_time_mark_list[4] = { 0 };	// Planned pedestrian time(In case some cars instertedand interrupt the pedestiran time)

	// Spillback info
	Road_Info my_road_info[4 * LANE_NUM_PER_DIRECTION];
	Road_Info* others_road_info[4 * LANE_NUM_PER_DIRECTION];
	//self.spillback_delay_record = [0]*(4*cfg.LANE_NUM_PER_DIRECTION)
	

	IntersectionManager();
	IntersectionManager(Coord id);

	void connect(const uint8_t& my_direction, IntersectionManager& target_intersection, const uint8_t& its_direction);
	Car_Info_In_Intersection get_car_info_for_route(const string& car_id);
	string check_in_my_region(string lane_id);
	void update_car(string car_id, string lane_id, double simu_step, char current_turn, char next_turn);
	void update_path(string car_id, char current_turn, char next_turn, uint8_t intersection_dir);
private:
	void set_round_lane();
};