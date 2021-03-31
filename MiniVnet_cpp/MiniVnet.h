#pragma once

#include "globalConst.h"
#include <map>
#include <tuple>
#include <vector>

// Record each time step on each node for updating database
class Car_in_Node_Record {
public:
	uint16_t time_stamp = 0;				// "Database time" that the car arrives
	Coord last_intersection_id = Coord(0, 0);
	string car_state = "";
	Car_in_database car_in_database;

	Car_in_Node_Record() {}
	Car_in_Node_Record(uint16_t in_time_stamp, Coord in_last_intersection_id,
		string in_car_state, Car_in_database in_car_in_database) :
		time_stamp(in_time_stamp), last_intersection_id(in_last_intersection_id),
		car_state(car_state), car_in_database(car_in_database) {}

};
// Record info of each node during routing
typedef tuple<Coord, uint8_t> Node_ID;
class Node_Record {
public:
	bool is_src = false;
	uint16_t arrival_time_stamp = 0;		// "Database time" that the car arrives
	Node_ID last_intersection_id;
	char turning = 'S';
	vector<Car_in_Node_Record> recordings;

	Node_Record() {}
	Node_Record(bool in_is_src, uint16_t arrival_time_stamp) : is_src(in_is_src), arrival_time_stamp(arrival_time_stamp) {}
};

class Node_in_Heap {
public:
	uint16_t current_arrival_time = 0;
	Node_ID current_node;
	double position_at_offset = 0;

	Node_in_Heap() {}
	Node_in_Heap(uint16_t arrival_time, Node_ID node, double pos_offset) : current_arrival_time(arrival_time), current_node(node), position_at_offset(pos_offset) {}
};
struct Compare_AT {
	bool operator()(Node_in_Heap const& node1, Node_in_Heap const& node2) {
		return node1.current_arrival_time > node2.current_arrival_time;
	}
};
class Node_in_Path {
public:
	char turning = 0;
	vector<Car_in_Node_Record> recordings;
	uint16_t time = 0;
	Node_in_Path() {}
	Node_in_Path(char in_turning, vector<Car_in_Node_Record> in_recordings, uint16_t in_time)
		: turning(in_turning), recordings(in_recordings), time(in_time) {}
};

