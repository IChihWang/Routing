#include "global.h"

Lane_Adviser::Lane_Adviser() {
	for (int lane_idx = 0; lane_idx < LANE_NUM_PER_DIRECTION; lane_idx++) {
		count_advised_not_secheduled_car_num[Trajectory_ID(lane_idx, 'L')] = 0;
		count_advised_not_secheduled_car_num[Trajectory_ID(lane_idx, 'R')] = 0;
		count_advised_not_secheduled_car_num[Trajectory_ID(lane_idx, 'S')] = 0;
	}
}

void Lane_Adviser::advise_lane(const Car& car) {

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

	if (direction == 0) {
		
	}
}