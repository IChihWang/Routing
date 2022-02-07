#pragma once

#ifndef GLOBAL_H
#define GLOBAL_H

#include <map>
#include <unordered_map>
#include <tuple>
#include <vector>
#include <shared_mutex>	// C++ 17
#include <algorithm>
#include <iostream>
#include <set>
#include <fstream>
#include "globalConst.h"
#include "element.h"
#include "LaneAdviser.h"
#include "MiniVnet.h"

using namespace std;

// Global variables
// Defined in element.cpp

// Defined in thread_worker.cpp
class Thread_Worker;
extern vector<Thread_Worker*> _thread_pool;
void routing_in_thread(Thread_Worker* thread_i);
void init_thread_pool();
void terminate_thread_pool();

// Defined in main.cpp
extern map<string, map<string, double> > inter_info;
string handle_request(string &in_str);
void read_load_adv_data();
void read_inter_info_data();
void read_inter_length_data();
ostream& operator<<(ostream& os, const Coord& coord_id);

extern ofstream _debug_file;

// Defined in CarRoute.cpp
#define OUTSIDE_MEC_MAP Coord(-1, -1)
extern map< Node_ID, double> _avg_map_delay;
void initial_avg_map();
void compute_avg_map();					// Abstract the district info (Average on each road)
void brief_route();						// Route for every car
void decide_tmp_destination();			// Acclocate temporary destination for cars within a district

// Defined in LoadBalancing.cpp
extern map< Coord, Coord> _intersection_MEC;	// Record the MEC ID for each intersection
extern map< Edge_ID, Coord> _roadseg_MEC;	// Record the MEC ID for each road segment
extern map< string, Coord>	_car_id_MEC_map;	// Record the MEC ID for each car
extern vector<Coord> _MEC_id_list;	// The list of ID of MECs
extern int _MEC_num_per_edge;		// Set 3 just for now for debugging
extern map< Coord, vector<Coord>> _MEC_intersection;	// Record the intersections that each MEC has
void initial_district_allocation();
void put_cars_into_districts();
void load_balancing(const vector<string>& new_car_ids);
void update_intersection_info_in_MEC();
void estimate_new_car_load(const vector<string>& new_car_ids);
pair<int, int> calculate_MEC_cost_info(Coord& MEC_id);	// return <V, E>


// Defined in MiniVnet.cpp
extern vector< map< Coord, Intersection* >* > _database;
extern map<string, Car> _car_dict;
extern vector< pair<int32_t, Intersection*> > _top_congested_intersections;

void add_intersection_to_reschedule_list(vector< pair<int32_t, Intersection*> >& district_top_congested_intersections);

void create_grid_network();
void add_time_step(double time_stamp);
void connect_intersections(map< Coord, Intersection* >& intersection_map);
void move_a_time_step();
void update_car(const string& car_id, const uint8_t& car_length, const string& src_intersection_id,
	const uint8_t& direction_of_src_intersection, const uint16_t& time_offset_step,
	const double& position_at_offset, const string& dst_node_id);
Intersection& get_intersection(const uint16_t current_arrival_time, const Coord& intersection_id);
vector<vector<reference_wrapper<Car>>> choose_car_to_thread_group(const Coord& MEC_id, vector<string> &new_car_ids, vector< pair<int32_t, Intersection*> >& district_top_congested_intersections);
void delete_car_from_database(Car& car);
void delete_car_from_database_id(string car_id);

// Single thread
map<string, string>& routing_with_groups(const Coord& MEC_id, const vector<vector<reference_wrapper<Car>>>& route_groups, map<string, string>& routes_dict);
// Multi threads (in thread_worker.cpp)
map<string, string>& routing_with_groups_thread(const Coord& MEC_id, const vector<vector<reference_wrapper<Car>>>& route_groups, map<string, string>& routes_dict);

map<string, vector<Node_in_Path>> routing(const Coord& MEC_id, const vector<reference_wrapper<Car>>& route_group, set< pair<uint16_t, Intersection*> >& thread_affected_intersections);
map<char, Node_ID> decide_available_turnings(Coord src_coord, uint8_t src_intersection_direction, Coord dst_coord, uint16_t additional_search_range);
void add_car_to_database(Car& target_car, const vector<Node_in_Path>& path_list, set< pair<uint16_t, Intersection*> >& thread_affected_intersections);
void testQ();


// Defined in LaneAdviser.cpp
tuple<double, double> get_Conflict_Region(Car_in_database car1, Car_in_database car2);
double get_Intertime(uint8_t lane, char turn);


#endif