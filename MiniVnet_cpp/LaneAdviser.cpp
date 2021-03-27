#include "global.h"

Lane_Adviser::Lane_Adviser() {
	for (int lane_idx = 0; lane_idx < LANE_NUM_PER_DIRECTION; lane_idx++) {
		count_advised_not_secheduled_car_num[Trajectory_ID(lane_idx, 'L')] = 0;
		count_advised_not_secheduled_car_num[Trajectory_ID(lane_idx, 'R')] = 0;
		count_advised_not_secheduled_car_num[Trajectory_ID(lane_idx, 'S')] = 0;
	}
}

uint8_t Lane_Adviser::advise_lane(const Car& car, const bool spillback_lane_advise_avoid[]) {
	uint8_t advise_lane = 0;
	
	return advise_lane;
}

void Lane_Adviser::update_Table_from_cars(const map<string, Car_in_database>& advising_car, const map<string, Car_in_database>& scheduling_cars, const map<string, Car_in_database>& sched_cars) {
	for (pair<string, Car_in_database> const& data_pair : sched_cars) {
		const Car_in_database &car = data_pair.second;
		update_Table(car, car.OT+car.D);
	}
	for (pair<string, Car_in_database> const& data_pair : scheduling_cars) {
		const Car_in_database& car = data_pair.second;
		update_Table(car, car.position/_V_MAX);

		count_advised_not_secheduled_car_num[Trajectory_ID(car.lane, car.current_turn)] += 1;
	}
	for (pair<string, Car_in_database> const& data_pair : advising_car) {
		const Car_in_database& car = data_pair.second;
		update_Table(car, car.position / _V_MAX);

		count_advised_not_secheduled_car_num[Trajectory_ID(car.lane, car.current_turn)] += 1;
	}
}

void Lane_Adviser::update_Table(const Car_in_database& car, double time) {
	int direction = car.lane / LANE_NUM_PER_DIRECTION;
	string lookup_key = to_string(car.lane% LANE_NUM_PER_DIRECTION) + car.current_turn;

	if (direction == 0) {
		for (map<char, uint8_t> occupied_boxes : lane_dict[lookup_key]) {
			if (time > timeMatrix[occupied_boxes['X']][occupied_boxes['Y']]) {
				timeMatrix[occupied_boxes['X']][occupied_boxes['Y']] = time;
			}
		}
	}
	else if (direction == 1) {
		for (map<char, uint8_t> occupied_boxes : lane_dict[lookup_key]) {
			if (time > timeMatrix[LANE_ADV_NUM - 1 - occupied_boxes['Y']][occupied_boxes['X']]) {
				timeMatrix[LANE_ADV_NUM - 1 - occupied_boxes['Y']][occupied_boxes['X']] = time;
			}
		}
	}
	else if (direction == 2) {
		for (map<char, uint8_t> occupied_boxes : lane_dict[lookup_key]) {
			if (time > timeMatrix[LANE_ADV_NUM - 1 - occupied_boxes['X']][occupied_boxes['Y']]) {
				timeMatrix[LANE_ADV_NUM - 1 - occupied_boxes['X']][occupied_boxes['Y']] = time;
			}
		}
	}
	else if (direction == 3) {
		for (map<char, uint8_t> occupied_boxes : lane_dict[lookup_key]) {
			if (time > timeMatrix[occupied_boxes['Y']][LANE_ADV_NUM - 1 - occupied_boxes['X']]) {
				timeMatrix[occupied_boxes['Y']][LANE_ADV_NUM - 1 - occupied_boxes['X']] = time;
			}
		}
	}
}