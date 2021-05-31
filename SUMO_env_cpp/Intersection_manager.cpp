#include "intersection_manager.h"
#include "Car.h"
#include "global.h"
#include <sstream>
#include <iomanip>
#include <algorithm>
#include <string>
#include <sstream>
using namespace std;

IntersectionManager::IntersectionManager() {
	for (uint8_t idx = 0; idx < 4 * LANE_NUM_PER_DIRECTION; idx++) {
		CC_last_cars_on_lanes[idx] = nullptr;
	}

	set_round_lane();
}

IntersectionManager::IntersectionManager(My_Coord id) : id(id) {
	stringstream ss;
	ss << std::setw(3) << std::setfill('0') << get<0>(id);
	id_str += ss.str();
	id_str += "_";
	ss.str("");
	ss << std::setw(3) << std::setfill('0') << get<1>(id);
	id_str += ss.str();

	for (uint8_t idx = 0; idx < 4 * LANE_NUM_PER_DIRECTION; idx++) {
		CC_last_cars_on_lanes[idx] = nullptr;
	}

	set_round_lane();
}

void IntersectionManager::connect(const uint8_t& my_direction, IntersectionManager& target_intersection, const uint8_t& its_direction) {
	//		2
	//	3   i   1
	//		0

	for (uint8_t lane_i = 0; lane_i < LANE_NUM_PER_DIRECTION; lane_i++) {
		uint8_t my_lane = my_direction * LANE_NUM_PER_DIRECTION + lane_i;
		uint8_t its_lane = its_direction * LANE_NUM_PER_DIRECTION + (LANE_NUM_PER_DIRECTION - lane_i - 1);

		others_road_info[my_lane] = &target_intersection.my_road_info[its_lane];
		target_intersection.others_road_info[its_lane] = &my_road_info[my_lane];
	}
}

void IntersectionManager::set_round_lane() {
	for (uint8_t i = 1; i < 5; i++) {
		for (uint8_t j = 0; j < LANE_NUM_PER_DIRECTION; j++) {
			string idx_str = id_str + '_' + to_string(i) + '_' + to_string(j);
			in_lanes.push_back(idx_str);
		}
	}
}

Car_Info_In_Intersection IntersectionManager::get_car_info_for_route(const string& car_id) {
	double time_offset = -1;
	string src_intersection_id = "";              // After offset, which intersection / node
	uint8_t direction_of_src_intersection = 0;
	uint8_t src_shift_num = 0;
	double position = car_list[car_id]->position;

	// Not yet enter Roadrunner
	if (car_list[car_id]->zone == "") {
		double diff_pos = position - TOTAL_LEN;
		time_offset = diff_pos / V_MAX;

		if (time_offset > ROUTING_PERIOD) {
			// Get the origin intersection id
			uint8_t lane = car_list[car_id]->lane;
			uint8_t direction = lane / LANE_NUM_PER_DIRECTION;

			direction_of_src_intersection = direction;
			src_intersection_id = id_str;
		}
		else {
			time_offset = -1;
		}
	}

	// 1. Not yet but about to enter the intersection region
	// 2. Already inside the intersection region
	if (time_offset == -1) {
		src_shift_num = 1;

		// Find lane of the car
		uint8_t lane = 0;
		if (car_list[car_id]->zone == "AZ") {
			lane = car_list[car_id]->desired_lane;
		}
		else {
			lane = car_list[car_id]->lane;
		}

		// Compute the time_offset
		if (!car_list[car_id]->is_scheduled) {
			// Delay is not computed yet
			time_offset = position / V_MAX;
			// Estimate by borrowing the known delay
			if (my_road_info[lane].car_delay_position.size() > 0) {
				time_offset += my_road_info[lane].car_delay_position[my_road_info[lane].car_delay_position.size() -1].delay;
			}
		}
		else {
			// Compute with the known delay
			time_offset = car_list[car_id]->OT + car_list[car_id]->D;
		}


		// Add the time in the intersection
		char turning = car_list[car_id]->current_turn;
		double time_in_inter = get_Intertime(lane, turning);
		time_offset += time_in_inter;  // time passing intersection

		// Start from next intersection
		if (time_offset <= ROUTING_PERIOD) {
			// About to exit the intersection
			// Not allowing scheduling for a while
			return Car_Info_In_Intersection(true);
		}
		else {

			uint8_t direction = lane / LANE_NUM_PER_DIRECTION;

			if (turning == 'S') {
				direction_of_src_intersection = direction;
			}
			else if (turning == 'L') {
				direction_of_src_intersection = (direction + 1) % 4;
			}
			else if (turning == 'R') {
				direction_of_src_intersection = (direction + 4 - 1) % 4;
			}

			string x = "";
			string y = "";

			stringstream ss;
			ss << std::setw(3) << std::setfill('0') << get<0>(id);
			x = ss.str();
			ss.str("");
			ss << std::setw(3) << std::setfill('0') << get<1>(id);
			y = ss.str();

			if (direction_of_src_intersection == 0) {
				stringstream sss;
				sss << std::setw(3) << std::setfill('0') << (get<1>(id) + 1);
				y = sss.str();
			}
			else if (direction_of_src_intersection == 1) {
				stringstream sss;
				sss << std::setw(3) << std::setfill('0') << (get<0>(id) - 1);
				x = sss.str();
			}
			else if (direction_of_src_intersection == 2) {
				stringstream sss;
				sss << std::setw(3) << std::setfill('0') << (get<1>(id) - 1);
				y = sss.str();
			}
			else if (direction_of_src_intersection == 3) {
				stringstream sss;
				sss << std::setw(3) << std::setfill('0') << (get<0>(id) + 1);
				x = sss.str();
			}

			src_intersection_id = x + "_" + y;
		}
	}

	int time_offset_step = int(time_offset / SCHEDULING_PERIOD);
	time_offset = double(time_offset_step + 1) * SCHEDULING_PERIOD - time_offset;
	double position_at_offset = TOTAL_LEN - time_offset * V_MAX;

	return Car_Info_In_Intersection(position_at_offset, time_offset_step, src_intersection_id, direction_of_src_intersection, src_shift_num);
}

string IntersectionManager::check_in_my_region(string lane_id) {
	if (find(in_lanes.begin(), in_lanes.end(), lane_id) != in_lanes.end()) {
		return "On my lane";
	}
	else {
		stringstream ss_car(lane_id);
		string x;
		getline(ss_car, x, '_');
		string y;
		getline(ss_car, y, '_');
		string lane_id_short = x + '_' + y;

		if (lane_id_short == (string(":") + id_str)) {
			return "In my intersection";
		}
		else {
			return "Not me";
		}
	}
}

void IntersectionManager::update_car(string car_id, string lane_id, double simu_step, char current_turn, char next_turn) {
	if (find(in_lanes.begin(), in_lanes.end(), lane_id) != in_lanes.end()) {
		// parse the lane_id
		stringstream ss_car(lane_id);
		string x;
		getline(ss_car, x, '_');
		string y;
		getline(ss_car, y, '_');
		string lane_direction_str;
		getline(ss_car, lane_direction_str, '_');
		uint8_t lane_direction = uint8_t(stoi(lane_direction_str));
		string lane_sub_idx_str;
		getline(ss_car, lane_sub_idx_str, '_');
		uint8_t lane_sub_idx = uint8_t(stoi(lane_sub_idx_str));
		
		uint8_t lane = uint8_t((4 - lane_direction) * LANE_NUM_PER_DIRECTION + (LANE_NUM_PER_DIRECTION - lane_sub_idx - 1));

		// Add car if the car is not in the list yet
		if (car_list.find(car_id) == car_list.end()) {
			// Gather the information of the new car
			uint8_t length = uint8_t(traci.vehicle.getLength(car_id));
			char turning = current_turn;

			Car* new_car_ptr = new Car(car_id, length, lane, turning, next_turn);
			new_car_ptr->Enter_T = simu_step - (traci.vehicle.getLanePosition(car_id)) / V_MAX;
			car_list[car_id] = new_car_ptr;

			traci.vehicle.setMaxSpeed(car_id, V_MAX);
			traci.vehicle.setSpeed(car_id, V_MAX);
		}

		update_path(car_id, current_turn, next_turn, lane_direction);
		car_list[car_id]->current_turn = current_turn;
		car_list[car_id]->next_turn = next_turn;

		// Set the position of each cars
		double position = traci.lane.getLength(lane_id) - traci.vehicle.getLanePosition(car_id);
		car_list[car_id]->position = position;

		if ((car_list[car_id]->zone == "") && (position <= TOTAL_LEN - car_list[car_id]->length-HEADWAY)) {
			car_list[car_id]->zone = "AZ";
			car_list[car_id]->zone_state = "AZ_not_advised";
		}
		else if ((car_list[car_id]->zone == "AZ") && (position <= PZ_LEN + GZ_LEN + BZ_LEN + CCZ_LEN)) {
			car_list[car_id]->zone = "PZ";
			car_list[car_id]->zone_state = "PZ_not_set";
		}

		else if ((car_list[car_id]->zone == "PZ") && (position <= GZ_LEN + BZ_LEN + CCZ_LEN)) {
			car_list[car_id]->zone = "GZ";
			car_list[car_id]->zone_state = "not_scheduled";
		}

		else if ((car_list[car_id]->zone == "GZ") && (position <= BZ_LEN + CCZ_LEN)) {
			car_list[car_id]->zone = "BZ";
		}

		else if ((car_list[car_id]->zone == "BZ") && (position <= CCZ_LEN)) {
			car_list[car_id]->zone = "CCZ";
		}

		car_list[car_id]->lane = lane;
	}
}

void IntersectionManager::update_path(string car_id, char current_turn, char next_turn, uint8_t intersection_dir) {
	uint16_t x_idx = get<0>(id);
	uint16_t y_idx = get<1>(id);

	uint8_t target_dir = 0;

	if (current_turn == 'R') {
		target_dir = ((intersection_dir + 4 - 1) + 1) % 4 + 1;		// +4 to prevent negative
	}
	else if (current_turn == 'S') {
		target_dir = intersection_dir;
	}
	else if (current_turn == 'L') {
		target_dir = ((intersection_dir + 4 - 1) - 1) % 4 + 1;		// +4 to prevent negative
	}

	if (target_dir == 1) {
		x_idx = x_idx + 1;
		y_idx = y_idx;
	}
	else if (target_dir == 2) {
		x_idx = x_idx;
		y_idx = y_idx - 1;
	}
	else if (target_dir == 3) {
		x_idx = x_idx - 1;
		y_idx = y_idx;
	}
	else if (target_dir == 4) {
		x_idx = x_idx;
		y_idx = y_idx + 1;
	}

	// Compute the intersection id
	string intersection_manager_id = "";
	stringstream ss;
	ss << std::setw(3) << std::setfill('0') << x_idx;
	intersection_manager_id += ss.str();
	intersection_manager_id += "_";
	ss.str("");
	ss << std::setw(3) << std::setfill('0') << y_idx;
	intersection_manager_id += ss.str();

	string target_edge = intersection_manager_id + "_" + to_string(target_dir);
	traci.vehicle.changeTarget(car_id, target_edge);
	traci.vehicle.setMaxSpeed(car_id, V_MAX);
	traci.vehicle.setColor(car_id, libsumo::TraCIColor(255, 255, 255));

	// Update dst lanes and info
	car_list[car_id]->set_turning(current_turn, next_turn);
}

void IntersectionManager::delete_car(string car_id) {
	if (car_list.find(car_id) != car_list.end()) {
		delete car_list[car_id];
		car_list.erase(car_id);
	}
}

void IntersectionManager::run(double simu_step) {
	// ===== Update the time OT =====
	for (const auto& [car_key, car_ptr] : car_list) {

		// Update when the car is scheduled
		car_ptr->OT -= _TIME_STEP;

#ifdef EXP_FUEL
		total_fuel_consumption.push_back(traci.vehicle.getFuelConsumption(car_key) * _TIME_STEP);
#endif

		if (car_ptr->is_spillback_strict == true) {
			traci.vehicle.setColor(car_key, libsumo::TraCIColor(255, 59, 59));
		}
		else if (car_ptr->is_spillback == true) {
			traci.vehicle.setColor(car_key, libsumo::TraCIColor(255, 153, 51));
		}
		else if (car_ptr->is_scheduled) {
			traci.vehicle.setColor(car_key, libsumo::TraCIColor(100, 250, 92));
		}
	}

	// ===== Entering the intersection (Record the cars) =====
	for (const auto& [car_id, car_ptr] : car_list) {
		string lane_id = traci.vehicle.getLaneID(car_id);
		if (find(in_lanes.begin(), in_lanes.end(), lane_id) == in_lanes.end()) {
			traci.vehicle.setSpeed(car_id, car_ptr->speed_in_intersection);

			/*
			if ((car_ptr->Leave_T == 0)) {
				cout << car_id << "  " << car_ptr->OT + car_ptr->D << " : " << car_ptr->OT << " " << car_ptr->D << endl;
			}
			*/

			car_ptr->Leave_T = simu_step;

#ifdef EXP_DELAY
			total_delays.push_back((car.Leave_T - car.Enter_T) - (TOTAL_LEN / V_MAX));
			total_delays_by_sche.push_back(car.D);
#endif		

			car_ptr->zone = "Intersection";
		}
	}

	// ===== Starting Cruise control
	vector<string> to_be_deleted_CC;
	for (const auto& [car_id, car_ptr] : pz_list) {
		if (car_ptr->position <= CCZ_LEN && car_ptr->is_scheduled == true) {
			to_be_deleted_CC.push_back(car_id);

			if (car_ptr->CC_state == "Preseting_done") {
				car_ptr->CC_state = "CruiseControl_ready";
			}
			else if (car_ptr->position <= CCZ_LEN) {
				to_be_deleted_CC.push_back(car_id);

				if ((car_ptr->CC_state == "") || (!(car_ptr->CC_state.find("Platoon") != string::npos || car_ptr->CC_state.find("Entering") != string::npos))) {
					car_ptr->CC_state = "Keep_Max_speed";
				}
			}
		}
	}
	for (const auto& car_id : to_be_deleted_CC) {
		pz_list.erase(car_id);
	}

	// ##############################################
	// Grouping the cars and schedule
	// Put here due to the thread handling
	schedule_period_count += _TIME_STEP;
	if (schedule_period_count > GZ_LEN / V_MAX - 1) {
		// Classify the cars for scheduler
		map<string, Car*> sched_car;
		map<string, Car*> n_sched_car;
		map<string, Car*> advised_n_sched_car;

		for (const auto& [car_id, car_ptr] : car_list) {
			if (car_ptr->zone == "GZ" || car_ptr->zone == "BZ" || car_ptr->zone == "CCZ" || car_ptr->zone == "Intersection") {
				if (car_ptr->zone_state == "not_scheduled") {
					n_sched_car[car_id] = car_ptr;
				}
				else {
					sched_car[car_id] = car_ptr;
					//traci.vehicle.setColor(car_id, libsumo::TraCIColor(100, 250, 92));
				}
			}
			else if (car_ptr->zone == "PZ" || car_ptr->zone == "AZ") {
				advised_n_sched_car[car_id] = car_ptr;
			}
		}
		for (const auto& [car_id, car_ptr] : n_sched_car) {
			car_ptr->is_scheduled = false;
			car_ptr->D = -1;
		}

		#ifdef PEDESTRIAN
		// Setting the pedestrian list
		for (uint8_t i = 0; i < 4; i++)
			is_pedestrian_list[i] = true;
		for (uint8_t direction = 0; direction < 4; direction++) {
			// Cancel the request if a pedestrian time has been scheduled
			if (is_pedestrian_list[direction] == true && (pedestrian_time_mark_list[direction] > 0)) {
				is_pedestrian_list[direction] = false;
			}
		}
		get_max_AT_direction(sched_car, is_pedestrian_list, pedestrian_time_mark_list);
		#endif

		scheduling(sched_car, n_sched_car, advised_n_sched_car);
		schedule_period_count = 0;
	}

	// ################################################
	// Set Max Speed in PZ
	vector<string> to_be_deleted_PZ;
	for (const auto& [car_id, car_ptr] : az_list) {
		if (car_ptr->zone == "PZ" && car_ptr->zone_state == "PZ_not_set") {
			traci.vehicle.setMinGap(car_id, HEADWAY);
			pz_list[car_id] = car_ptr;
			to_be_deleted_PZ.push_back(car_id);

			// Take over the speed control from the car
			traci.vehicle.setSpeedMode(car_id, 0);
			if (car_ptr->CC_state == "")
				car_ptr->CC_state = "Preseting_start";

			// Cancel the auto gap
			traci.vehicle.setLaneChangeMode(car_id, 0);
			uint8_t lane = car_ptr->lane;
			car_ptr->desired_lane = lane;
			uint8_t lane_sub_idx = (LANE_NUM_PER_DIRECTION - lane % LANE_NUM_PER_DIRECTION - 1);

			// Stay on its lane
			traci.vehicle.changeLane(car_id, lane_sub_idx, 1.0);
			car_ptr->zone_state = "PZ_set";
		}
	}

	for (const string& car_id : to_be_deleted_PZ) {
		az_list.erase(car_id);
	}
	to_be_deleted_PZ.clear();

	// Set Max Speed in PZ
	for (const auto& [car_id, car_ptr] : pz_list) {
		if (car_ptr->zone == "PZ" && car_ptr->zone_state == "PZ_set") {
			uint8_t lane = car_ptr->lane;
			car_ptr->desired_lane = car_ptr->lane;

			uint8_t out_sub_lane = (LANE_NUM_PER_DIRECTION - lane % LANE_NUM_PER_DIRECTION - 1);
			car_ptr->dst_lane = int(car_ptr->out_direction * LANE_NUM_PER_DIRECTION + out_sub_lane);     // Destination lane before next lane change

			// Stay on its lane
			traci.vehicle.changeLane(car_id, out_sub_lane, 1.0);
		}
	}


	// ##########################################
	// Cruse Control

	// Start to let cars control itself once it enters the CCZ
	// Each car perform their own Cruise Control behavior
	vector<pair<string, Car*>> sorted_cars_list;
	for (auto& [car_id, car_ptr] : car_list) {
		sorted_cars_list.push_back(make_pair(car_id, car_ptr));
	}

	sort(sorted_cars_list.begin(), sorted_cars_list.end(), [](pair<string, Car*> a, pair<string, Car*> b) -> bool {return a.second->position < b.second->position; });
	for (auto& [car_id, car_ptr] : sorted_cars_list) {
		// Cars perform their own CC
		if (car_ptr->zone != "" && car_ptr->zone != "Intersection")
			car_ptr->handle_CC_behavior(car_list);
	}

	// ################################################
	// Change lane in AZ

	// Check whether there is a spillback
	uint32_t accumulate_car_len_lane[4 * LANE_NUM_PER_DIRECTION] = {0};
	bool spillback_lane_advise_avoid[4 * LANE_NUM_PER_DIRECTION] = {0};

	for (const auto& [car_id, car_ptr] : car_list) {
		Car& car = *car_ptr;
		uint8_t lane_idx = car.dst_lane;
		uint8_t lane_changed_to_idx = car.dst_lane_changed_to;

		if (others_road_info[lane_idx] != nullptr)
			accumulate_car_len_lane[lane_idx] += (car.length + HEADWAY);
		if (others_road_info[lane_changed_to_idx] != nullptr)
			accumulate_car_len_lane[lane_changed_to_idx] += (car.length + HEADWAY);
		if (car.is_spillback == true)
			spillback_lane_advise_avoid[lane_idx] = true;
	}
	for (uint8_t lane_idx = 0; lane_idx < 4 * LANE_NUM_PER_DIRECTION; lane_idx++) {
		if (others_road_info[lane_idx] != nullptr) {
			if (accumulate_car_len_lane[lane_idx] >= others_road_info[lane_idx]->avail_len)
				spillback_lane_advise_avoid[lane_idx] = true;
		}
	}

	for (const auto& [car_id, car_ptr] : car_list) {
		Car& car = *car_ptr;
		if (car.zone == "AZ" and car.zone_state == "AZ_not_advised") {
			az_list[car_id] = car_ptr;

			traci.vehicle.setMinGap(car_id, 3);
			traci.vehicle.setLaneChangeMode(car_id, 784);

			double time_in_AZ = 9999.91;
			uint8_t advised_lane = lane_advisor.advise_lane(*(car_list[car_id]), spillback_lane_advise_avoid);

			traci.vehicle.changeLane(car_id, advised_lane, time_in_AZ);
			car.desired_lane = int((LANE_NUM_PER_DIRECTION - advised_lane - 1) + (car.lane/LANE_NUM_PER_DIRECTION)*LANE_NUM_PER_DIRECTION);
			car.zone_state = "AZ_advised";
		}
		else if (car.zone == "AZ" && car.zone_state == "AZ_advised" && car.position <= PZ_LEN + GZ_LEN + BZ_LEN + CCZ_LEN + CCZ_ACC_LEN) {
			pair<string, double> leader_tuple = traci.vehicle.getLeader(car.id, 0);
			if (leader_tuple.first != "") {
				if (car_list.find(leader_tuple.first) != car_list.end()) {
					string front_car_ID = leader_tuple.first;
					Car* front_car_ptr = car_list[front_car_ID];
					double front_distance = leader_tuple.second;

					double my_speed = traci.vehicle.getSpeed(car.id);
					double front_speed = traci.vehicle.getSpeed(front_car_ptr->id);
					double min_catch_up_time = (my_speed - front_speed) / MAX_ACC;
					double min_distance = (my_speed - front_speed) * min_catch_up_time;

					double min_gap = max(double(HEADWAY), min_distance + HEADWAY);
					traci.vehicle.setMinGap(car_id, min_gap);
				}
			}
		}
	}

	// ##########################################
	// Update the road info after actions
	double car_accumulate_len_lane[LANE_NUM_PER_DIRECTION * 4] = { 0 };
	fill_n(car_accumulate_len_lane, 4 * LANE_NUM_PER_DIRECTION, (CCZ_DEC2_LEN + CCZ_ACC_LEN));
	double delay_lane[LANE_NUM_PER_DIRECTION * 4] = { 0 };
	double car_position_with_delay_lane[LANE_NUM_PER_DIRECTION * 4] = { 0 };
	map<uint8_t, vector<Car_Delay_Position_Record>>	lane_car_delay_position;

	for (const auto& [car_id, car_ptr] : sorted_cars_list) {
		Car& car = *car_ptr;

		if (car.zone == "Intersection") {
			// Skip cars that has entered the intersection
			continue;
		}

		uint8_t lane = car.lane;
		car_accumulate_len_lane[lane] += car.length + HEADWAY;
		if (car.is_scheduled) {
			lane_car_delay_position[lane].push_back(Car_Delay_Position_Record(car_accumulate_len_lane[lane], car.D, car.id));
		}

		if (car.position > TOTAL_LEN - AZ_LEN && lane != car.desired_lane) {
			car_accumulate_len_lane[car.desired_lane] += car.length + HEADWAY;
		}

		if (car.position > car_position_with_delay_lane[lane] && car.is_scheduled) {
			car_position_with_delay_lane[lane] = car.position;
			delay_lane[lane] = car.D;
		}
	}

	for (uint8_t lane_idx = 0; lane_idx < 4 * LANE_NUM_PER_DIRECTION; lane_idx++) {
		my_road_info[lane_idx].avail_len = TOTAL_LEN - car_accumulate_len_lane[lane_idx] - HEADWAY;
		my_road_info[lane_idx].delay = delay_lane[lane_idx];


		my_road_info[lane_idx].car_delay_position = lane_car_delay_position[lane_idx];
	}
}

#ifdef PEDESTRIAN
void IntersectionManager::get_max_AT_direction(const vector<Car*>& sched_car, const bool* is_pedestrian_list, double* pedestrian_time_mark_list) {
	double max_AT[4];
	for (uint8_t i = 0; i < 4; i++)
		max_AT[i] = 0;

	for (const auto& car_ptr : sched_car) {
		// Compute the direction of entering / exiting for the intersection
		uint8_t in_dir = car_ptr->in_direction;
		uint8_t out_dir = car_ptr->out_direction;

		// Compute arrival time at the exiting point and entering point
		double in_AT = car_ptr->OT + car_ptr->D + car_ptr->length / car_ptr->speed_in_intersection;
		double out_AT = car_ptr->OT + car_ptr->D + car_ptr->length / V_MAX + get_Intertime(car_ptr->lane, car_ptr->current_turn);

		// Find max and update
		if (in_AT > max_AT[in_dir])
			max_AT[in_dir] = in_AT;
		if (out_AT > max_AT[out_dir])
			max_AT[out_dir] = out_AT;
	}

	for (uint8_t direction = 0; direction < 4; direction++)
		if (pedestrian_time_mark_list[direction] <= 0)
			if (is_pedestrian_list[direction] == false)
				pedestrian_time_mark_list[direction] = -1;
			else {
				pedestrian_time_mark_list[direction] = max_AT[direction];
			}
}
#endif