#include "ortools/linear_solver/linear_solver.h"
#include <algorithm>
#include "global.h"
#include "intersection_manager.h"
#include "Car.h"

using namespace operations_research;

double IntersectionManager::scheduling(map<string, Car*>& sched_car, map<string, Car*>& n_sched_car, map<string, Car*>& advised_n_sched_car) {

	Roadrunner_P(sched_car, n_sched_car, advised_n_sched_car);

}

void IntersectionManager::Roadrunner_P(map<string, Car*>& old_cars, map<string, Car*>& new_cars, map<string, Car*>& advised_n_sched_car) {

	// part 1: build the solver
	MPSolver solver("Linearptr", MPSolver::GLOP_LINEAR_PROGRAMMING);
	const double infinity = solver.infinity();

	// part 2: claim variables
	map<string, const MPVariable*> D_solver_variables;

	// part 3: claim parameters
	uint16_t pre_accumulate_car_len_lane[4 * LANE_NUM_PER_DIRECTION] = {};

	// Compute accumulated car len
	for (const auto& [car_id, old_car_ptr] : old_cars) {
		Car& old_car = *old_car_ptr;
		pre_accumulate_car_len_lane[old_car.dst_lane] += (old_car.length + HEADWAY);
		pre_accumulate_car_len_lane[old_car.dst_lane_changed_to] += (old_car.length + HEADWAY);

	}

	map<string, Car*> sorted_scheduling_cars_list;
	for (auto& [car_id, car_ptr] : new_cars) {
		sorted_scheduling_cars_list[car_id] = car_ptr;
	}

	sort(sorted_scheduling_cars_list.begin(), sorted_scheduling_cars_list.end(), [](Car* a, Car* b) -> bool {return a->position < b->position; });
	double head_of_line_blocking_position[4 * LANE_NUM_PER_DIRECTION];
	fill_n(head_of_line_blocking_position, 4 * LANE_NUM_PER_DIRECTION, UINT16_MAX);
	int32_t accumulate_car_len[4 * LANE_NUM_PER_DIRECTION];
	fill_n(accumulate_car_len, 4 * LANE_NUM_PER_DIRECTION, INT32_MIN);
	double recorded_delay[4 * LANE_NUM_PER_DIRECTION];
	fill_n(recorded_delay, 4 * LANE_NUM_PER_DIRECTION, 0);

	for (uint8_t i = 0; i < 4 * LANE_NUM_PER_DIRECTION; i++) {
		if (others_road_info[i] != nullptr) {
			accumulate_car_len[i] = pre_accumulate_car_len_lane[i] - (others_road_info[i])->avail_len + CAR_MAX_LEN + HEADWAY + CCZ_ACC_LEN;
			recorded_delay[i] = max(others_road_info[i]->delay, spillback_delay_record[i]);	// To record the dispatch speed
		}
	}

	for (auto& [car_id, car_ptr] : sorted_scheduling_cars_list) {
		Car& car = *car_ptr;
		car.is_spillback = false;
		car.is_spillback_strict = false;

		uint8_t dst_lane_idx = car.dst_lane;
		uint8_t dst_lane_changed_to_idx = car.dst_lane_changed_to;
		uint8_t lane_idx = car.lane;

		if (others_road_info[dst_lane_idx] != nullptr) {
			if (car.position > head_of_line_blocking_position[lane_idx]) {
				new_cars.erase(car_id);    // Blocked by the car at the front
				continue;
			}

			vector<Car_Delay_Position_Record>& dst_car_delay_position = (others_road_info[dst_lane_idx])->car_delay_position;
			accumulate_car_len[dst_lane_idx] += (car.length + HEADWAY);
			double spillback_delay = 0;















			if (car.position > head_of_line_blocking_position[lane_idx]) {
				// The car is blocked by front car, whose sheduling is prosponed.
				car.is_spillback_strict = true;
				car.is_spillback = true;

				// remove car from scheduling_cars
				uint32_t idx_in_scheduling_cars = -1;
				for (uint32_t idx = 0; idx < (uint32_t)scheduling_cars.size(); idx++) {
					if (car.id.compare(scheduling_cars[idx].id) == 0) {
						idx_in_scheduling_cars = idx;
						break;
					}
				}
				string deleted_car_id = car.id;
				scheduling_cars.erase(scheduling_cars.begin() + idx_in_scheduling_cars);

				if (deleted_car_id.compare(target_car.id) == 0) {
					target_car.D = SCHEDULE_POSPONDED;
					return;
				}

				continue;
			}

			accumulate_car_len[dst_lane_idx] += (car.length + _HEADWAY);
			if (accumulate_car_len[dst_lane_idx] > 0) {
				const vector<Car_Delay_Position_Record>& dst_car_delay_position = (*(others_road_info[dst_lane_idx]))->car_delay_position;

				car.is_spillback = true;
				if (dst_car_delay_position.size() < 1 || (double(accumulate_car_len[dst_lane_idx]) + CAR_MAX_LEN + _HEADWAY > dst_car_delay_position.back().position)) {
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
					double back_position = dst_car_delay_position[compare_dst_car_idx].position;
					double spillback_delay_multiply_factor = back_delay / back_position;
					spillback_delay = accumulate_car_len[dst_lane_idx] * spillback_delay_multiply_factor;
				}
			}

			for (uint8_t lane_i = 0; lane_i < LANE_NUM_PER_DIRECTION; lane_i++) {
				uint8_t other_lane_idx = dst_lane_idx / LANE_NUM_PER_DIRECTION * LANE_NUM_PER_DIRECTION + lane_i;

				if (other_lane_idx != dst_lane_idx) {
					accumulate_car_len[other_lane_idx] += (car.length + _HEADWAY);
					double spillback_delay_dst_lane_changed_to = 0;
					const vector<Car_Delay_Position_Record>& dst_car_delay_position = (*(others_road_info[other_lane_idx]))->car_delay_position;

					if (accumulate_car_len[other_lane_idx] > 0) {
						car.is_spillback = true;

						if (dst_car_delay_position.size() < 1 || (double(accumulate_car_len[other_lane_idx]) + double(CAR_MAX_LEN) + _HEADWAY > dst_car_delay_position.back().position)) {
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
							double back_position = dst_car_delay_position[compare_dst_car_idx].position;
							double spillback_delay_multiply_factor = back_delay / back_position;
							double spillback_delay_alter = accumulate_car_len[other_lane_idx] * spillback_delay_multiply_factor;
							spillback_delay = max(spillback_delay, spillback_delay_alter);
						}
					}
				}
			}

			if (car.is_spillback_strict == true) {
				// remove car from scheduling_cars
				uint32_t idx_in_scheduling_cars = -1;
				for (uint32_t idx = 0; idx < (uint32_t)scheduling_cars.size(); idx++) {
					if (car.id.compare(scheduling_cars[idx].id) == 0) {
						idx_in_scheduling_cars = idx;
						break;
					}
				}

				string deleted_car_id = car.id;
				scheduling_cars.erase(scheduling_cars.begin() + idx_in_scheduling_cars);

				if (car.position < head_of_line_blocking_position[lane_idx]) {
					head_of_line_blocking_position[lane_idx] = car.position;
				}

				if (deleted_car_id.compare(target_car.id) == 0) {
					target_car.D = SCHEDULE_POSPONDED;
					return;
				}
			}
			else {
				if (car.current_turn == 'S') {
					MPVariable* const temp_D = solver.MakeNumVar(spillback_delay, infinity, "d" + car.id);
					D_solver_variables[car.id] = temp_D;
				}
				else {
					double min_d = (2 * CCZ_DEC2_LEN / (double(_V_MAX) + _TURN_SPEED)) - (CCZ_DEC2_LEN / _V_MAX);
					MPVariable* const temp_D = solver.MakeNumVar(max(min_d, spillback_delay), infinity, "d" + car.id);
					D_solver_variables[car.id] = temp_D;
				}
			}
		}
		else {
			if (car.position > head_of_line_blocking_position[lane_idx]) {
				// remove car from scheduling_cars
				uint32_t idx_in_scheduling_cars = -1;
				for (uint32_t idx = 0; idx < (uint32_t)scheduling_cars.size(); idx++) {
					if (car.id.compare(scheduling_cars[idx].id) == 0) {
						idx_in_scheduling_cars = idx;
						break;
					}
				}

				string deleted_car_id = car.id;
				scheduling_cars.erase(scheduling_cars.begin() + idx_in_scheduling_cars);

				if (deleted_car_id.compare(target_car.id) == 0) {
					target_car.D = SCHEDULE_POSPONDED;
					return;
				}
			}
			else {
				if (car.current_turn == 'S') {
					MPVariable* const temp_D = solver.MakeNumVar(0, infinity, "d" + car.id);
					D_solver_variables[car.id] = temp_D;
				}
				else {
					double min_d = (2 * CCZ_DEC2_LEN / (double(_V_MAX) + _TURN_SPEED)) - (CCZ_DEC2_LEN / _V_MAX);
					MPVariable* const temp_D = solver.MakeNumVar(min_d, infinity, "d" + car.id);
					D_solver_variables[car.id] = temp_D;
				}
			}
		}
	}


	// part 4: set constrain (10) (Car on same lane, rear-end collision avoidance)
	// (1) old car and new car
	for (const Car_in_database& new_car : scheduling_cars) {
		for (const auto& [car_id, old_car] : *sched_cars) {
			
			double bound = old_car.length / old_car.speed_in_intersection + (old_car.OT + old_car.D);
			bound += _HEADWAY / old_car.speed_in_intersection;
			if (new_car.current_turn == 'S' && old_car.current_turn != 'S') {
				bound += (double(_V_MAX) - _TURN_SPEED) * (CCZ_DEC2_LEN) / (_V_MAX * (double(_V_MAX) + _TURN_SPEED));
			}
			bound = bound - new_car.OT;

			MPConstraint* const tmp_conts = solver.MakeRowConstraint(bound, infinity);
			tmp_conts->SetCoefficient(D_solver_variables[new_car.id], 1);
		}
	}

	// (2) two new cars
	for (uint32_t i = 0; i < (uint32_t)scheduling_cars.size(); i++) {
		for (uint32_t j = i+1; j < (uint32_t)scheduling_cars.size(); j++) {
			Car_in_database* car_a_ptr = &(scheduling_cars[i]);
			Car_in_database* car_b_ptr = &(scheduling_cars[j]);
			
			// Ensure car_a_ptr->OT > car_b_ptr->OT
			if (car_a_ptr->OT < car_b_ptr->OT) {
				swap(car_a_ptr, car_b_ptr);
			}

			double bound = car_b_ptr->length / car_b_ptr->speed_in_intersection - car_a_ptr->OT + car_b_ptr->OT;
			bound += _HEADWAY / car_b_ptr->speed_in_intersection;
			if (car_a_ptr->current_turn == 'S' && car_b_ptr->current_turn != 'S') {
				bound += (double(_V_MAX) - _TURN_SPEED) * (CCZ_DEC2_LEN) / (double(_V_MAX) * (double(_V_MAX) + _TURN_SPEED));
			}
			MPConstraint* const tmp_conts = solver.MakeRowConstraint(bound, infinity);
			tmp_conts->SetCoefficient(D_solver_variables[car_a_ptr->id], 1);
			tmp_conts->SetCoefficient(D_solver_variables[car_b_ptr->id], -1);
		}
	}

	// part 5: set constrain (11) (two new cars in the intersection)
	for (uint32_t i = 0; i < (uint32_t)scheduling_cars.size(); i++) {
		for (uint32_t j = i + 1; j < (uint32_t)scheduling_cars.size(); j++) {
			Car_in_database& car_i = scheduling_cars[i];
			Car_in_database& car_j = scheduling_cars[j];

			if (car_i.lane == car_j.lane) {
				continue;
			}

			tuple<double, double> conflict_region_data = get_Conflict_Region(car_i, car_j);
			if (conflict_region_data != tuple<double, double>(0, 0)) {
				double tau_S1_S2 = get<0>(conflict_region_data);
				double tau_S2_S1 = get<1>(conflict_region_data);

				MPVariable* const flag = solver.MakeIntVar(0, 1, string("flag_new_new_") + to_string(i) + "_" + to_string(j));

				double bound_2 = -car_j.OT + car_i.OT + tau_S1_S2 - LARGE_NUM;
				MPConstraint* const tmp_conts2 = solver.MakeRowConstraint(bound_2, infinity);
				tmp_conts2->SetCoefficient(D_solver_variables[car_i.id], -1);
				tmp_conts2->SetCoefficient(D_solver_variables[car_j.id], 1);
				tmp_conts2->SetCoefficient(flag, -LARGE_NUM);

				double bound_1 = -car_i.OT + car_j.OT + tau_S2_S1;
				MPConstraint* const tmp_conts1 = solver.MakeRowConstraint(bound_1, infinity);
				tmp_conts1->SetCoefficient(D_solver_variables[car_i.id], 1);
				tmp_conts1->SetCoefficient(D_solver_variables[car_j.id], -1);
				tmp_conts1->SetCoefficient(flag, LARGE_NUM);
			}
		}
	}

	// part 6: set constrain (12) (one new car and one old car in the intersection)
	for (const Car_in_database& new_car : scheduling_cars) {
		for (const auto& [car_id, old_car] : *sched_cars) {

			if (new_car.lane == old_car.lane) {
				continue;
			}

			tuple<double, double> conflict_region_data = get_Conflict_Region(old_car, new_car);
			if (conflict_region_data != tuple<double, double>(0, 0)) {
				double tau_S1_S2 = get<0>(conflict_region_data);
				double tau_S2_S1 = get<1>(conflict_region_data);

				MPVariable* const flag = solver.MakeIntVar(0, 1, string("flag_old_new_") + old_car.id + "_" + new_car.id);

				double bound_3 = -old_car.D - old_car.OT + new_car.OT + tau_S1_S2 - LARGE_NUM;
				MPConstraint* const tmp_conts3 = solver.MakeRowConstraint(bound_3, infinity);
				tmp_conts3->SetCoefficient(D_solver_variables[new_car.id], -1);
				tmp_conts3->SetCoefficient(flag, -LARGE_NUM);

				double bound_4 = old_car.D + old_car.OT - new_car.OT + tau_S2_S1;
				MPConstraint* const tmp_conts4 = solver.MakeRowConstraint(bound_3, infinity);
				tmp_conts4->SetCoefficient(D_solver_variables[new_car.id], 1);
				tmp_conts4->SetCoefficient(flag, LARGE_NUM);
			}
		}
	}

	// part 7: set objective
	MPObjective* const objective = solver.MutableObjective();
	for (const Car_in_database& new_car : scheduling_cars) {
		objective->SetCoefficient(D_solver_variables[new_car.id], 1);
	}
	objective->SetMinimization();

	// part 8: solve the problem
	const MPSolver::ResultStatus result_status = solver.Solve();

	// Check that the problem has an optimal solution.
	if (result_status != MPSolver::FEASIBLE && result_status != MPSolver::OPTIMAL) {
		cout << "The problem has no solution!" << endl;
	}

	// Update the delays
	target_car.D = D_solver_variables[target_car.id]->solution_value();
}