#include "global.h"

using namespace std;

Intersection::Intersection() : others_road_info() {}

Intersection::Intersection(const Coord &in_coordinate) : others_road_info() {
	id = in_coordinate;
}

void Intersection::connect(const uint8_t& my_direction, Intersection& target_intersection, const uint8_t& its_direction) {
	//		2
	//	3   i   1
	//		0

	for (uint8_t lane_i = 0; lane_i < LANE_NUM_PER_DIRECTION; lane_i++) {
		uint8_t my_lane = my_direction * LANE_NUM_PER_DIRECTION + lane_i;
		uint8_t its_lane = its_direction * LANE_NUM_PER_DIRECTION + (LANE_NUM_PER_DIRECTION - lane_i-1);

		others_road_info[my_lane] = &target_intersection.my_road_info[its_lane];
		target_intersection.others_road_info[its_lane] = &my_road_info[my_lane];
	}
}

void Intersection::delete_car_from_intersection(const Car& car, const string& type) {
	if (type.compare("lane_advising") == 0){
		Car_in_database car_in_database = advising_car[car.id];
		advising_car.erase(car.id);
		update_my_spillback_info(car_in_database);
		AZ_accumulated_size -= (car.length + _HEADWAY);
	}
	else if (type.compare("scheduling") == 0) {
		Car_in_database car_in_database = scheduling_cars[car.id];
		scheduling_cars.erase(car.id);
		update_my_spillback_info(car_in_database);
		GZ_accumulated_size -= (car.length + _HEADWAY);
	}
	else if (type.compare("scheduled") == 0) {
		Car_in_database car_in_database = sched_cars[car.id];
		sched_cars.erase(car.id);
		update_my_spillback_info(car_in_database);
		GZ_accumulated_size -= (car.length + _HEADWAY);
	}

}

void Intersection::update_my_spillback_info(Car_in_database& car_in_database) {
	// TODO: update spillback info
}

uint8_t Intersection::advise_lane(const Car& car) {
	Lane_Adviser lane_adviser;

	// Update advising table with existing cars
	lane_adviser.update_Table_from_cars(advising_car, scheduling_cars, sched_cars);

	// Check whether there is spillback
	uint16_t accumulate_car_len_lane[4 * LANE_NUM_PER_DIRECTION] = {};
	bool spillback_lane_advise_avoid[4 * LANE_NUM_PER_DIRECTION] = {};

	// Calculate accumulated length
	for (pair<string, Car_in_database> const& data_pair : sched_cars) {
		const Car_in_database& car_in_database = data_pair.second;
		uint8_t lane_idx = car_in_database.lane;

		if (others_road_info[lane_idx] != nullptr) {
			accumulate_car_len_lane[lane_idx] += (car_in_database.length + _HEADWAY);
		}
	}
	for (pair<string, Car_in_database> const& data_pair : scheduling_cars) {
		const Car_in_database& car_in_database = data_pair.second;
		uint8_t lane_idx = car_in_database.lane;

		if (others_road_info[lane_idx] != nullptr) {
			accumulate_car_len_lane[lane_idx] += (car_in_database.length + _HEADWAY);
		}
	}
	for (pair<string, Car_in_database> const& data_pair : advising_car) {
		const Car_in_database& car_in_database = data_pair.second;
		uint8_t lane_idx = car_in_database.lane;

		if (others_road_info[lane_idx] != nullptr) {
			accumulate_car_len_lane[lane_idx] += (car_in_database.length + _HEADWAY);
		}
	}

	// mark the lane to avoid
	for (uint8_t lane_i = 0; lane_i < 4 * LANE_NUM_PER_DIRECTION; lane_i++) {
		if (others_road_info[lane_i] != nullptr) {
			if (accumulate_car_len_lane[lane_i] >= others_road_info[lane_i]->avail_len) {
				spillback_lane_advise_avoid[lane_i] = true;
			}
		}
	}

	uint8_t advised_lane = lane_adviser.advise_lane(car, spillback_lane_advise_avoid);

	return advised_lane;
}

tuple<bool, double> Intersection::is_GZ_full(const Car& car, const double& position_at_offset) {
	// Tell if the intersection is fulland the car have to wait
	if (GZ_accumulated_size > _GZ_BZ_CCZ_len) {
		return tuple<bool, double>(false, position_at_offset);
	}
	else if (position_at_offset > GZ_accumulated_size) {
		return tuple<bool, double>(true, position_at_offset);
	}
	else {
		return tuple<bool, double>(true, GZ_accumulated_size + car.length + _HEADWAY);
	}
}



Car_in_database::Car_in_database(const string in_id, const uint8_t in_length) {
	id = in_id;
	length = in_length;
}

void Car_in_database::update_dst_lane_and_data() {
	int in_direction = lane / LANE_NUM_PER_DIRECTION;
	int out_direction = 0;

	if (current_turn == 'S') {
		out_direction = (in_direction + 2) % 4;
	}
	else if (current_turn == 'R') {
		out_direction = (in_direction + 1) % 4;
	}
	else if (current_turn == 'L') {
		out_direction = (in_direction - 1) % 4;
	}

	uint8_t out_sub_lane = (LANE_NUM_PER_DIRECTION - lane % LANE_NUM_PER_DIRECTION - 1);
	dst_lane = int(out_direction * LANE_NUM_PER_DIRECTION + out_sub_lane);     // Destination lane before next lane change
	dst_lane_changed_to = int(out_direction * LANE_NUM_PER_DIRECTION + out_sub_lane);  // Destination lane after next lane change

	// Determine the speed in the intersection
	if (current_turn == 'S') {
		speed_in_intersection = _V_MAX;
	}
	else {
		speed_in_intersection = _TURN_SPEED;
	}

}

Car::Car(const string in_id, const uint8_t in_length, const Coord in_dst_coord) {
	id = in_id;
	length = in_length;
	dst_coord = in_dst_coord;
}