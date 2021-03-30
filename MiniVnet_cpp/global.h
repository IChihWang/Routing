#ifndef GLOBAL_H
#define GLOBAL_H

#include <map>
#include <tuple>
#include <vector>
#include <shared_mutex>	// C++ 17
#include <algorithm>
#include "globalConst.h"
#include "element.h"
#include "LaneAdviser.h"

using namespace std;

// Global variables
// Defined in element.cpp

// Defined in main.cpp
extern map<string, map<string, double> > inter_info;
string handle_request(string &in_str);


void read_load_adv_data();
void read_inter_info_data();
void read_inter_length_data();

extern vector< map< Coord, Intersection > > _database;
extern map<string, Car> _car_dict;

extern shared_mutex _database_g_mutex;
//extern unique_lock<shared_mutex> _database_wLock;
//extern shared_lock<shared_mutex> _database_rLock;

void create_grid_network();
void add_time_step();
void connect_intersections(map< Coord, Intersection >& intersection_map);
void move_a_time_step();
void update_car(const string& car_id, const uint8_t& car_length, const string& src_intersection_id,
	const uint8_t& direction_of_src_intersection, const uint16_t& time_offset_step,
	const double& position_at_offset, const string& dst_node_id);

vector<vector<reference_wrapper<Car>>> choose_car_to_thread_group(vector<string> &new_car_ids, vector<string> &old_car_ids);
void delete_car_from_database(Car& car);
void delete_car_from_database_id(string car_id);

void routing_with_groups(vector<vector<reference_wrapper<Car>>> route_groups, map<string, string> routes_dict);
void routing(vector<reference_wrapper<Car>>& route_group);

void testQ();


// Defined in LaneAdviser.cpp
tuple<double, double> get_Conflict_Region(Car_in_database car1, Car_in_database car2);
double get_Intertime(uint8_t lane, char turn);

#endif