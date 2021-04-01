#include "global.h"
#include <cassert>

using namespace std;

Intersection::Intersection() : others_road_info() {
	sched_cars = new map<string, Car_in_database>;
	scheduling_cars = new map<string, Car_in_database>;
	advising_car = new map<string, Car_in_database>;

	for (uint8_t i = 0; i < 4 * LANE_NUM_PER_DIRECTION; i++) {
		my_road_info[i] = new Road_Info;
	}
}
Intersection::Intersection(const Coord &in_coordinate) : others_road_info() {
	sched_cars = new map<string, Car_in_database>;
	scheduling_cars = new map<string, Car_in_database>;
	advising_car = new map<string, Car_in_database>;

	for (uint8_t i = 0; i < 4 * LANE_NUM_PER_DIRECTION; i++) {
		my_road_info[i] = new Road_Info;
	}

	id = in_coordinate;
}
Intersection::~Intersection() {
}

void Intersection::my_own_destructure() {
	for (const pair<string, Car*>& car_data : stored_cars) {
		Car* car_ptr = car_data.second;
		delete_myself_from_car_record(*car_ptr);
	}

	delete sched_cars;
	delete scheduling_cars;
	delete advising_car;
	for (uint8_t i = 0; i < 4 * LANE_NUM_PER_DIRECTION; i++) {
		delete my_road_info[i];
	}
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

void Intersection::delete_car_from_intersection(Car& car, const string& type) {
	if (type.compare("lane_advising") == 0){
		Car_in_database car_in_database = (*advising_car)[car.id];
		(*advising_car).erase(car.id);
		update_my_spillback_info(car_in_database);
		AZ_accumulated_size -= (car.length + _HEADWAY);
	}
	else if (type.compare("scheduling") == 0) {
		Car_in_database car_in_database = (*scheduling_cars)[car.id];
		(*scheduling_cars).erase(car.id);
		update_my_spillback_info(car_in_database);
		GZ_accumulated_size -= (car.length + _HEADWAY);
	}
	else if (type.compare("scheduled") == 0) {
		Car_in_database car_in_database = (*sched_cars)[car.id];
		(*sched_cars).erase(car.id);
		update_my_spillback_info(car_in_database);
		GZ_accumulated_size -= (car.length + _HEADWAY);
	}

	stored_cars.erase(car.id);
	delete_myself_from_car_record(car);

	assert(AZ_accumulated_size >= 0);
	assert(GZ_accumulated_size >= 0);
}
#include <iostream>
void Intersection::delete_myself_from_car_record(Car& car) {
	std::cout << "   ?!? " << car.records_intersection_in_database[this] << endl;
	car.records_intersection_in_database.erase(this);
	std::cout << "   dd ?!? " << car.records_intersection_in_database[this] << endl;
}

void Intersection::update_my_spillback_info(const Car_in_database& car) {
	const uint8_t& lane_idx = car.lane;
	cout << car.id << "   " << (int)lane_idx << endl;
	(my_road_info[lane_idx])->avail_len = _TOTAL_LEN - (GZ_accumulated_size + AZ_accumulated_size + _HEADWAY);

	// Filter out cars on the same lane
	vector<reference_wrapper<const Car_in_database>> scheduled_car_list;
	for (const pair<const string, Car_in_database>& car_data : (*sched_cars)) {
		const Car_in_database& sched_car = car_data.second;
		if (sched_car.lane == lane_idx) {
			scheduled_car_list.push_back(sched_car);
		}
	}
	sort(scheduled_car_list.begin(), scheduled_car_list.end(), [](Car_in_database a, Car_in_database b) -> bool {return a.OT < b.OT; });

	// Store cars into the list
	vector<Car_Delay_Position_Record> car_delay_position;
	double car_accumulate_len_lane = CCZ_DEC2_LEN + CCZ_ACC_LEN;
	for (const Car_in_database& sched_car : scheduled_car_list) {
		car_accumulate_len_lane += double(sched_car.length) + _HEADWAY;
		car_delay_position.push_back(Car_Delay_Position_Record(car_accumulate_len_lane, sched_car.D));
	}

	(my_road_info[lane_idx])->car_delay_position = car_delay_position;

}

uint8_t Intersection::advise_lane(const Car& car) {
	Lane_Adviser lane_adviser;

	// Update advising table with existing cars
	lane_adviser.update_Table_from_cars((*advising_car), (*scheduling_cars), (*sched_cars));

	// Check whether there is spillback
	uint16_t accumulate_car_len_lane[4 * LANE_NUM_PER_DIRECTION] = {};
	bool spillback_lane_advise_avoid[4 * LANE_NUM_PER_DIRECTION] = {};

	// Calculate accumulated length
	for (const pair<string, Car_in_database>& data_pair : (*sched_cars)) {
		const Car_in_database& car_in_database = data_pair.second;
		uint8_t lane_idx = car_in_database.lane;

		if (others_road_info[lane_idx] != nullptr) {
			accumulate_car_len_lane[lane_idx] += (car_in_database.length + _HEADWAY);
		}
	}
	for (const pair<string, Car_in_database>& data_pair : (*scheduling_cars)) {
		const Car_in_database& car_in_database = data_pair.second;
		uint8_t lane_idx = car_in_database.lane;

		if (others_road_info[lane_idx] != nullptr) {
			accumulate_car_len_lane[lane_idx] += (car_in_database.length + _HEADWAY);
		}
	}
	for (const pair<string, Car_in_database>& data_pair : (*advising_car)) {
		const Car_in_database& car_in_database = data_pair.second;
		uint8_t lane_idx = car_in_database.lane;

		if (others_road_info[lane_idx] != nullptr) {
			accumulate_car_len_lane[lane_idx] += (car_in_database.length + _HEADWAY);
		}
	}

	// mark the lane to avoid
	for (uint8_t lane_i = 0; lane_i < 4 * LANE_NUM_PER_DIRECTION; lane_i++) {
		if (others_road_info[lane_i] != nullptr) {
			if (accumulate_car_len_lane[lane_i] >= (*(others_road_info[lane_i]))->avail_len) {
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
void Intersection::add_sched_car(Car_in_database car, Car& target_car) {
	(*sched_cars)[car.id] = car;
	stored_cars[car.id] = &target_car;
	GZ_accumulated_size += (car.length + _HEADWAY);
	update_my_spillback_info(car);
}
void Intersection::add_scheduling_cars(Car_in_database car, Car& target_car) {
	(*scheduling_cars)[car.id] = car;
	stored_cars[car.id] = &target_car;
	GZ_accumulated_size += (car.length + _HEADWAY);
	update_my_spillback_info(car);
}
void Intersection::add_advising_car(Car_in_database car, Car& target_car) {
	(*advising_car)[car.id] = car;
	stored_cars[car.id] = &target_car;
	AZ_accumulated_size += (car.length + _HEADWAY);
	update_my_spillback_info(car);
}





Car_in_database::Car_in_database(const string in_id, const uint8_t in_length) {
	id = in_id;
	length = in_length;
}

void Car_in_database::update_dst_lane_and_data() {
	int8_t in_direction = lane / LANE_NUM_PER_DIRECTION;
	int8_t out_direction = 0;

	if (current_turn == 'S') {
		out_direction = (in_direction + 2) % 4;
	}
	else if (current_turn == 'R') {
		out_direction = (in_direction + 1) % 4;
	}
	else if (current_turn == 'L') {
		out_direction = (in_direction+4 - 1) % 4;	// +4 to make it positive
	}
	
	uint8_t out_sub_lane = (LANE_NUM_PER_DIRECTION - lane % LANE_NUM_PER_DIRECTION - 1);
	dst_lane = uint8_t(out_direction * LANE_NUM_PER_DIRECTION + out_sub_lane);     // Destination lane before next lane change
	dst_lane_changed_to = uint8_t(out_direction * LANE_NUM_PER_DIRECTION + out_sub_lane);  // Destination lane after next lane change

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