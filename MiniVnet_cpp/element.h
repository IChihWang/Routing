#ifndef ELEMENT_H
#define ELEMENT_H

#include <map>
#include <tuple>
#include <vector>
#include <queue>
#include <string>
#include "globalConst.h"
using namespace std;

#ifndef COORD
#define COORD
typedef tuple<int, int> Coord;
#endif


class Car;
class Car_in_database;
class Intersection;
class Lane_Adviser;

class Car_Delay_Position_Record {
public:
	double position = 0;
	double delay = 0;

	Car_Delay_Position_Record(){}
	Car_Delay_Position_Record(double in_position, double in_delay): position(in_position), delay(in_delay){}
};

class Road_Info {
public:
	uint16_t avail_len = _TOTAL_LEN;
	vector<Car_Delay_Position_Record> car_delay_position;
};

class Intersection {
public:
	Coord id;
	uint16_t AZ_accumulated_size = 0;
	uint16_t GZ_accumulated_size = 0;

	// Stored cars
	map<string, Car_in_database> sched_cars;
	map<string, Car_in_database> scheduling_cars;
	map<string, Car_in_database> advising_car;

	// Spillback info
	Road_Info my_road_info[4 * LANE_NUM_PER_DIRECTION];
	Road_Info* others_road_info[4 * LANE_NUM_PER_DIRECTION];

	Intersection();
	Intersection(const Coord& in_coordinate);
	void connect(const uint8_t& my_direction, Intersection& target_intersection, const uint8_t& its_direction);
	uint8_t advise_lane(const Car& car);
	tuple<bool, double> is_GZ_full(const Car& car, const double& position_at_offset);
	double scheduling(Car& car);

	// Thread Critical
	void add_sched_car(const Car_in_database& car);
	void add_scheduling_cars(const Car_in_database& car);
	void add_advising_car(const Car_in_database& car);
	void delete_car_from_intersection(const Car& car, const string& type);
	void update_my_spillback_info(const Car_in_database& car);

private:
	void Roadrunner_P(vector<Car_in_database>& copied_scheduling_cars, Car& target_car);
};

class Car_in_database {
public:
	string id = "";
	uint8_t lane = 0;
	uint8_t length = 0;
	char current_turn = 'S';
	double position = 0;


	// temporary variables for routing
	double D = 0;
	double OT = 0;
	uint8_t dst_lane = 0;
	uint8_t dst_lane_changed_to = 0;
	double speed_in_intersection = 0;

	bool is_spillback = false;
	bool is_spillback_strict = false;


	Car_in_database() {};
	Car_in_database(const string in_id, const uint8_t in_length);

	void update_dst_lane_and_data();
};

class Node_in_Path;
class Car : public Car_in_database {
public:
	Coord dst_coord;
	vector< tuple<string, Intersection> > records_intersection_in_database;
	
	// temporary variables for routing
	Coord src_coord;
	uint8_t direction_of_src_intersection = 0;
	uint16_t time_offset_step = 0;
	double position_at_offset = 0;

	double traveling_time = 0;

	vector<Node_in_Path> path_data;

	Car() {};
	Car(const string in_id, const uint8_t in_length, const Coord in_dst_coord);
};

#endif