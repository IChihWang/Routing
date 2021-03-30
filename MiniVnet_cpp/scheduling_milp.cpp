#include "ortools/linear_solver/linear_solver.h"
#include <algorithm>
#include "global.h"

using namespace operations_research;

double Intersection::scheduling(Car& target_car) {

	target_car.OT = (double)target_car.position / _V_MAX;
	target_car.D = NOT_SCHEDULED;

	vector<Car_in_database> copied_scheduling_cars;
	for (const pair<string, Car_in_database>& data : scheduling_cars) {
		Car_in_database car = data.second;
		car.D = NOT_SCHEDULED;
		copied_scheduling_cars.push_back(car);
	}
	copied_scheduling_cars.push_back(target_car);

	Roadrunner_P(copied_scheduling_cars, target_car);

	// Copy copied_scheduling_cars back to scheduling_cars (Update intersection state)
	// TODO: thread_safe
	for (Car_in_database car : copied_scheduling_cars) {
		string car_id = car.id;
		scheduling_cars[car_id] = car;
	}

	// Return AT
	return SCHEDULE_POSPONDED;	//-1
}

void Intersection::Roadrunner_P(vector<Car_in_database>& scheduling_cars, Car& target_car) {
	// Include target_car into scheduling cars
	vector<reference_wrapper<Car_in_database>> scheduling_cars_list;
	scheduling_cars_list.push_back(target_car);
	for (Car_in_database& car : scheduling_cars) {
		scheduling_cars_list.push_back(car);
	}
	
	// part 1: calculate OT
	for (Car_in_database& car : scheduling_cars) {
		double OT = car.position / _V_MAX;
		car.OT = OT + SUMO_TIME_ERR;
	}

	// part 2: build the solver
	unique_ptr<MPSolver> solver(MPSolver::CreateSolver("SCIP"));

	// part 3: claim parameters
	uint16_t pre_accumulate_car_len_lane[4 * LANE_NUM_PER_DIRECTION] = {};

	// Compute accumulated car len
	for (const pair<string, Car_in_database>& old_car_data : sched_cars) {
		const Car_in_database& old_car = old_car_data.second;
		uint8_t lane_base_idx = old_car.dst_lane / LANE_NUM_PER_DIRECTION * LANE_NUM_PER_DIRECTION;

		for (uint8_t i = 0; i < LANE_NUM_PER_DIRECTION; i++) {
			pre_accumulate_car_len_lane[lane_base_idx + i] += old_car.length + _HEADWAY;
		}
	}

	sort(scheduling_cars_list.begin(), scheduling_cars_list.end(), [](Car_in_database a, Car_in_database b) -> bool {return a.position < b.position; });
	uint16_t head_of_line_blocking_position[4 * LANE_NUM_PER_DIRECTION];
	fill_n(head_of_line_blocking_position, 4 * LANE_NUM_PER_DIRECTION, UINT16_MAX);
	int32_t accumulate_car_len[4 * LANE_NUM_PER_DIRECTION];
	fill_n(accumulate_car_len, 4 * LANE_NUM_PER_DIRECTION, INT32_MIN);

	for (uint8_t i = 0; i < 4 * LANE_NUM_PER_DIRECTION; i++) {
		if (others_road_info[i] != nullptr) {
			accumulate_car_len[i] = pre_accumulate_car_len_lane[i] - others_road_info[i]->avail_len + CAR_MAX_LEN + _HEADWAY + CCZ_ACC_LEN;
		}
	}

	for (Car_in_database& car : scheduling_cars_list) {
		car.is_spillback = false;
		car.is_spillback_strict = false;

		uint8_t dst_lane_idx = car.dst_lane;
		uint8_t lane_idx = car.lane;

		if (others_road_info[dst_lane_idx] != nullptr) {
			double spillback_delay = 0;

			if (car.position > head_of_line_blocking_position[lane_idx]) {
				// The car is blocked by front car, whose sheduling is prosponed.
				car.is_spillback_strict = true;
				car.is_spillback = true;
				scheduling_cars.erase(remove(scheduling_cars.begin(), scheduling_cars.end(), car), scheduling_cars.end());

				if (car.id.compare(target_car.id) == 0) {
					target_car.D = SCHEDULE_POSPONDED;
					return;
				}

				continue;
			}

			accumulate_car_len[dst_lane_idx] += (car.length + _HEADWAY);
			if (accumulate_car_len[dst_lane_idx] > 0) {
				const vector<Car_Delay_Position_Record>& dst_car_delay_position = others_road_info[dst_lane_idx]->car_delay_position;

				car.is_spillback = true;
				if (dst_car_delay_position.size() < 1 || (accumulate_car_len[dst_lane_idx] + CAR_MAX_LEN + _HEADWAY > dst_car_delay_position.back().position)) {
					// Skip because no records is found
					car.is_spillback_strict = true;
				}
				else {
					// Find the position in the list to compare
					uint32_t compare_dst_car_idx = 0;
					for (uint32_t dst_car_idx = 0; dst_car_idx < (uint32_t)dst_car_delay_position.size(); dst_car_idx++) {
						if (accumulate_car_len[dst_car_idx] < dst_car_delay_position[dst_car_idx].position) {
							compare_dst_car_idx = dst_car_idx;
							break;
						}
					}

					double back_delay = dst_car_delay_position[compare_dst_car_idx].delay;
					uint32_t back_position = dst_car_delay_position[compare_dst_car_idx].position;
					double spillback_delay_multiply_factor = back_delay / back_position;
					spillback_delay = accumulate_car_len[dst_lane_idx] * spillback_delay_multiply_factor;
				}
			}

			for (uint8_t lane_i = 0; lane_i < LANE_NUM_PER_DIRECTION; lane_i++) {
				uint8_t other_lane_idx = dst_lane_idx / LANE_NUM_PER_DIRECTION * LANE_NUM_PER_DIRECTION + lane_i;

				if (other_lane_idx != dst_lane_idx) {
					accumulate_car_len[other_lane_idx] += (car.length + _HEADWAY);
					double spillback_delay_dst_lane_changed_to = 0;
					const vector<Car_Delay_Position_Record>& dst_car_delay_position = others_road_info[other_lane_idx]->car_delay_position;

					if (accumulate_car_len[other_lane_idx] > 0) {
						car.is_spillback = true;

						if (dst_car_delay_position.size() < 1 || (accumulate_car_len[other_lane_idx] + CAR_MAX_LEN + _HEADWAY > dst_car_delay_position.back().position)) {
							// Skip because no records is found
							car.is_spillback_strict = true;
						}
						else {
							// Find the position in the list to compare
							uint32_t compare_dst_car_idx = 0;
							for (uint32_t dst_car_idx = 0; dst_car_idx < (uint32_t)dst_car_delay_position.size(); dst_car_idx++) {
								if (accumulate_car_len[other_lane_idx] < dst_car_delay_position[other_lane_idx].position) {
									compare_dst_car_idx = dst_car_idx;
									break;
								}
							}

							double back_delay = dst_car_delay_position[compare_dst_car_idx].delay;
							uint32_t back_position = dst_car_delay_position[compare_dst_car_idx].position;
							double spillback_delay_multiply_factor = back_delay / back_position;
							double spillback_delay_alter = accumulate_car_len[other_lane_idx] * spillback_delay_multiply_factor;
							spillback_delay = max(spillback_delay, spillback_delay_alter);
						}
					}


				}
			}
		}
	}
}