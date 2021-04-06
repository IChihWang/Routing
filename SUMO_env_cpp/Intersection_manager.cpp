#include "intersection_manager.h"
#include <sstream>
#include <iomanip>
using namespace std;

IntersectionManager::IntersectionManager() {
	for (uint8_t idx = 0; idx < 4 * LANE_NUM_PER_DIRECTION; idx++) {
		CC_last_cars_on_lanes[idx] = nullptr;
	}

	set_round_lane();
}

IntersectionManager::IntersectionManager(Coord id) : id(id) {
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
	double position = car_list[car_id].position;

	// Not yet enter Roadrunner
	if (car_list[car_id].zone.compare("") == 0) {
		double diff_pos = position - TOTAL_LEN;
		time_offset = diff_pos / V_MAX;

		if (time_offset > ROUTING_PERIOD) {
			// Get the origin intersection id
			uint8_t lane = car_list[car_id].lane;
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
		if (car_list[car_id].zone.compare("AZ") == 0) {
			lane = car_list[car_id].desired_lane;
		}
		else {
			lane = car_list[car_id].lane;
		}

		// Compute the time_offset
		if (!car_list[car_id].is_scheduled) {
			// Delay is not computed yet
			time_offset = position / V_MAX;
			// Estimate by borrowing the known delay
			if (my_road_info[lane].car_delay_position.size() > 0) {
				time_offset += my_road_info[lane].car_delay_position[-1].delay;
			}
		}
		else {
			// Compute with the known delay
			time_offset = car_list[car_id].OT + car_list[car_id].D;
		}

		// Add the time in the intersection
		char turning = car_list[car_id].current_turn;
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
				direction_of_src_intersection = (direction - 1) % 4;
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

	uint16_t time_offset_step = int(time_offset / SCHEDULING_PERIOD);
	time_offset = double(time_offset_step + 1) * SCHEDULING_PERIOD - time_offset;
	double position_at_offset = TOTAL_LEN - time_offset * V_MAX;

	return Car_Info_In_Intersection(position_at_offset, time_offset_step, src_intersection_id, direction_of_src_intersection, src_shift_num);
}