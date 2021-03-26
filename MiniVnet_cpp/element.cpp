#include "global.h"

using namespace std;

Intersection::Intersection(const tuple<int, int> &in_coordinate) {
	id = in_coordinate;
}

void Intersection::delete_car_from_database(const Car& car, const string& type) {
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

Car::Car(const string in_id, const uint8_t in_length, const tuple<int, int> in_dst_coord) {
	id = in_id;
	length = in_length;
	dst_coord = in_dst_coord;
}