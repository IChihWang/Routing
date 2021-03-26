#ifndef ELEMENT_H
#define ELEMENT_H

#include <map>
#include <tuple>
#include <vector>
#include <queue>
#include <string>
using namespace std;


class Car;
class Car_in_database;
class Intersection;

typedef tuple<int, int> Coord;

class Intersection {
public:
	Coord id;
	uint16_t AZ_accumulated_size = 0;
	uint16_t GZ_accumulated_size = 0;

	map<string, Car_in_database> sched_cars;
	map<string, Car_in_database> scheduling_cars;
	map<string, Car_in_database> advising_car;

	Intersection() {}
	Intersection(const Coord& in_coordinate);
	void delete_car_from_database(const Car& car, const string& type);
	void update_my_spillback_info(Car_in_database &car_in_database);
	uint8_t advise_lane(const Car& car);
};

class Car_in_database {
public:
	string id = "";
	uint8_t lane = 0;
	uint8_t length = 0;
	char current_turn = 'S';
	uint16_t position = 0;


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

	//self.path_data = None

	Car() {};
	Car(const string in_id, const uint8_t in_length, const Coord in_dst_coord);
};

#endif