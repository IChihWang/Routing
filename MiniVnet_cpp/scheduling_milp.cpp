#include "ortools/linear_solver/linear_solver.h"
#include <algorithm>
#include "global.h"

using namespace operations_research;

double Intersection::scheduling(Car& target_car) {
	shared_lock<shared_mutex> rLock(rwlock_mutex);

	target_car.OT = (double)target_car.position / _V_MAX;
	target_car.D = NOT_SCHEDULED;

	vector<Car_in_database> copied_scheduling_cars;
	for (auto& [car_id, car] : *scheduling_cars) {
		car.D = NOT_SCHEDULED;
		copied_scheduling_cars.push_back(car);
	}
	copied_scheduling_cars.push_back(target_car);

	Roadrunner_P(copied_scheduling_cars, target_car);

	if (target_car.D == SCHEDULE_POSPONDED) {
		return SCHEDULE_POSPONDED;
	}
	else {
		double car_exiting_time = target_car.OT + target_car.D;
		car_exiting_time += get_Intertime(target_car.lane, target_car.current_turn);

		return car_exiting_time;
	}
}

void Intersection::Roadrunner_P(vector<Car_in_database>& scheduling_cars, Car& target_car) {
			
	// part 1: build the solver
	unique_ptr<MPSolver> solver(MPSolver::CreateSolver("SCIP"));
	const double infinity = solver->infinity();

	// part 2: claim variables
	map<string, const MPVariable*> D_solver_variables;

	// part 3: claim parameters
	uint16_t pre_accumulate_car_len_lane[4 * LANE_NUM_PER_DIRECTION] = {0};

	// Compute accumulated car len
	const map<string, Car_in_database>& my_sched_cars = (*sched_cars);
	for (const auto& [car_id, old_car] : my_sched_cars) {
		uint8_t lane_base_idx = old_car.dst_lane / LANE_NUM_PER_DIRECTION * LANE_NUM_PER_DIRECTION;

		for (uint8_t i = 0; i < LANE_NUM_PER_DIRECTION; i++) {
			pre_accumulate_car_len_lane[lane_base_idx + i] += old_car.length + _HEADWAY;
		}
	}

	vector<Car_in_database> sorted_scheduling_cars_list;
	for (auto& car : scheduling_cars) {
		sorted_scheduling_cars_list.push_back(car);
	}

	sort(sorted_scheduling_cars_list.begin(), sorted_scheduling_cars_list.end(), [](Car_in_database a, Car_in_database b) -> bool {return a.position < b.position; });
	double head_of_line_blocking_position[4 * LANE_NUM_PER_DIRECTION];
	fill_n(head_of_line_blocking_position, 4 * LANE_NUM_PER_DIRECTION, UINT16_MAX);
	int32_t accumulate_car_len[4 * LANE_NUM_PER_DIRECTION];
	fill_n(accumulate_car_len, 4 * LANE_NUM_PER_DIRECTION, INT32_MIN);

	for (uint8_t i = 0; i < 4 * LANE_NUM_PER_DIRECTION; i++) {
		if (others_road_info[i] != nullptr) {
			accumulate_car_len[i] = pre_accumulate_car_len_lane[i] - (*(others_road_info[i]))->avail_len + CAR_MAX_LEN + _HEADWAY + CCZ_ACC_LEN;
		}
	}
	cout << "scheduling  ==============================  " << get<0>(id) << "," << get<1>(id) << endl;
	for (Car_in_database& car : sorted_scheduling_cars_list) {
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
				if (dst_car_delay_position.size() < 1 || (double(accumulate_car_len[dst_lane_idx]) > dst_car_delay_position.back().position)) {
					// Skip because no records is found
					car.is_spillback_strict = true;
				}
				else {
					// Find the position in the list to compare
					uint32_t compare_dst_car_idx = 0;
					for (uint32_t dst_car_idx = 0; dst_car_idx < (uint32_t)dst_car_delay_position.size(); dst_car_idx++) {
						if (accumulate_car_len[dst_lane_idx] < dst_car_delay_position[dst_car_idx].position) {
							compare_dst_car_idx = dst_car_idx;
							break;
						}
					}

					double back_delay = dst_car_delay_position[compare_dst_car_idx].ET - car.OT;
					double back_position = dst_car_delay_position[compare_dst_car_idx].position;
					double spillback_delay_multiply_factor = back_delay / back_position;
					//spillback_delay = accumulate_car_len[dst_lane_idx] * spillback_delay_multiply_factor;
					spillback_delay = back_delay;
				}
			}

			for (uint8_t lane_i = 0; lane_i < LANE_NUM_PER_DIRECTION; lane_i++) {
				uint8_t other_lane_idx = dst_lane_idx / LANE_NUM_PER_DIRECTION * LANE_NUM_PER_DIRECTION + lane_i;

				if (other_lane_idx != dst_lane_idx) {
					
					double spillback_delay_dst_lane_changed_to = 0;
					const vector<Car_Delay_Position_Record>& dst_car_delay_position = (*(others_road_info[other_lane_idx]))->car_delay_position;
					accumulate_car_len[other_lane_idx] += (car.length + _HEADWAY);

					if (accumulate_car_len[other_lane_idx] > 0) {
						car.is_spillback = true;

						if (dst_car_delay_position.size() < 1 || (double(accumulate_car_len[other_lane_idx]) > dst_car_delay_position.back().position)) {
							// Skip because no records is found
							car.is_spillback_strict = true;
						}
						else {
							// Find the position in the list to compare
							uint32_t compare_dst_car_idx = 0;
							for (uint32_t dst_car_idx = 0; dst_car_idx < (uint32_t)dst_car_delay_position.size(); dst_car_idx++) {
								if (accumulate_car_len[other_lane_idx] < dst_car_delay_position[dst_car_idx].position) {
									compare_dst_car_idx = dst_car_idx;
									break;
								}
							}

							double back_delay = dst_car_delay_position[compare_dst_car_idx].ET - car.OT;
							double back_position = dst_car_delay_position[compare_dst_car_idx].position;
							double spillback_delay_multiply_factor = back_delay / back_position;
							//double spillback_delay_alter = accumulate_car_len[other_lane_idx] * spillback_delay_multiply_factor;
							double spillback_delay_alter = back_delay;
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
				
				if (car.position < head_of_line_blocking_position[lane_idx]) {
					head_of_line_blocking_position[lane_idx] = car.position;
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
					MPVariable* const temp_D = solver->MakeNumVar(spillback_delay, infinity, "d" + car.id);
					D_solver_variables[car.id] = temp_D;
				}
				else {
					double min_d = (2 * CCZ_DEC2_LEN / (double(_V_MAX) + _TURN_SPEED)) - (CCZ_DEC2_LEN / _V_MAX);
					MPVariable* const temp_D = solver->MakeNumVar(max(min_d, spillback_delay), infinity, "d" + car.id);
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
					MPVariable* const temp_D = solver->MakeNumVar(0, infinity, "d" + car.id);
					D_solver_variables[car.id] = temp_D;
				}
				else {
					double min_d = (2 * CCZ_DEC2_LEN / (double(_V_MAX) + _TURN_SPEED)) - (CCZ_DEC2_LEN / _V_MAX);
					MPVariable* const temp_D = solver->MakeNumVar(min_d, infinity, "d" + car.id);
					D_solver_variables[car.id] = temp_D;
				}
			}
		}
	}


	// part 4: set constrain (10) (Car on same lane, rear-end collision avoidance)
	// (1) old car and new car
	for (const Car_in_database& new_car : scheduling_cars) {
		for (const auto& [car_id, old_car] : *sched_cars) {
			if (new_car.lane == old_car.lane) {
				double bound = old_car.length / old_car.speed_in_intersection + (old_car.OT + old_car.D);
				bound += _HEADWAY / old_car.speed_in_intersection;
				if (new_car.current_turn == 'S' && old_car.current_turn != 'S') {
					bound += (double(_V_MAX) - _TURN_SPEED) * (CCZ_DEC2_LEN) / (_V_MAX * (double(_V_MAX) + _TURN_SPEED));
				}
				bound = bound - new_car.OT;

				MPConstraint* const tmp_conts = solver->MakeRowConstraint(bound, infinity);
				tmp_conts->SetCoefficient(D_solver_variables[new_car.id], 1);
			}
		}
	}

	// (2) two new cars
	for (uint32_t i = 0; i < (uint32_t)scheduling_cars.size(); i++) {
		for (uint32_t j = i+1; j < (uint32_t)scheduling_cars.size(); j++) {
			Car_in_database* car_a_ptr = &(scheduling_cars[i]);
			Car_in_database* car_b_ptr = &(scheduling_cars[j]);
			
			if (car_a_ptr->lane == car_b_ptr->lane) {
				// Ensure car_a_ptr->OT > car_b_ptr->OT
				if (car_a_ptr->OT < car_b_ptr->OT) {
					swap(car_a_ptr, car_b_ptr);
				}

				double bound = car_b_ptr->length / car_b_ptr->speed_in_intersection - car_a_ptr->OT + car_b_ptr->OT;
				bound += _HEADWAY / car_b_ptr->speed_in_intersection;
				if (car_a_ptr->current_turn == 'S' && car_b_ptr->current_turn != 'S') {
					bound += (double(_V_MAX) - _TURN_SPEED) * (CCZ_DEC2_LEN) / (double(_V_MAX) * (double(_V_MAX) + _TURN_SPEED));
				}
				MPConstraint* const tmp_conts = solver->MakeRowConstraint(bound, infinity);
				tmp_conts->SetCoefficient(D_solver_variables[car_a_ptr->id], 1);
				tmp_conts->SetCoefficient(D_solver_variables[car_b_ptr->id], -1);

			}
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

				MPVariable* const flag = solver->MakeIntVar(0, 1, string("flag_new_new_") + to_string(i) + "_" + to_string(j));

				double bound_2 = -car_j.OT + car_i.OT + tau_S1_S2 - LARGE_NUM;
				MPConstraint* const tmp_conts2 = solver->MakeRowConstraint(bound_2, infinity);
				tmp_conts2->SetCoefficient(D_solver_variables[car_i.id], -1);
				tmp_conts2->SetCoefficient(D_solver_variables[car_j.id], 1);
				tmp_conts2->SetCoefficient(flag, -LARGE_NUM);

				double bound_1 = -car_i.OT + car_j.OT + tau_S2_S1;
				MPConstraint* const tmp_conts1 = solver->MakeRowConstraint(bound_1, infinity);
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

			tuple<double, double> conflict_region_data = get_Conflict_Region(new_car, old_car);
			if (conflict_region_data != tuple<double, double>(0, 0)) {
				double tau_S1_S2 = get<0>(conflict_region_data);
				double tau_S2_S1 = get<1>(conflict_region_data);

				MPVariable* const flag = solver->MakeIntVar(0, 1, string("flag_old_new_") + old_car.id + "_" + new_car.id);

				double bound_3 = -old_car.D - old_car.OT + new_car.OT + tau_S1_S2 - LARGE_NUM;
				MPConstraint* const tmp_conts3 = solver->MakeRowConstraint(bound_3, infinity);
				tmp_conts3->SetCoefficient(D_solver_variables[new_car.id], -1);
				tmp_conts3->SetCoefficient(flag, -LARGE_NUM);

				double bound_4 = old_car.D + old_car.OT - new_car.OT + tau_S2_S1;
				MPConstraint* const tmp_conts4 = solver->MakeRowConstraint(bound_4, infinity);
				tmp_conts4->SetCoefficient(D_solver_variables[new_car.id], 1);
				tmp_conts4->SetCoefficient(flag, LARGE_NUM);
			}
		}
	}

	// part 7: set objective
	MPObjective* const objective = solver->MutableObjective();
	for (const Car_in_database& new_car : scheduling_cars) {
		objective->SetCoefficient(D_solver_variables[new_car.id], 1);
	}
	objective->SetMinimization();


	
	
	cout << "scheduling size: " << (int)scheduling_cars.size() << "  old car size: " << (int)sched_cars->size() << endl;
	for (const Car_in_database& new_car : scheduling_cars) {
		cout << new_car.id << " | " << new_car.OT << " | " << new_car.position << " | " << (int)new_car.length << " | " << (int)new_car.lane << " | " << new_car.current_turn << endl;
	}
	cout << "------------------  " << endl;
	for (const auto& [car_id, old_car] : *sched_cars) {
		cout << car_id << " | " << old_car.OT << " | " << old_car.position << " | " << (int)old_car.length << " | " << (int)old_car.lane << " | " << old_car.D << " | " << old_car.current_turn << endl;
	}

	/*
	if (get<0>(id) == 2 && get<1>(id) == 1) {
		if (target_car.id
		("car_898") == 0 || target_car.id.compare("car_805") == 0 || target_car.id.compare("car_816") == 0 || target_car.id.compare("car_745") == 0) {
			cout << "===============  " << get<0>(id) << "," << get<1>(id) << endl;
			cout << "scheduling size: " << (int)scheduling_cars.size() << "  old car size: " << (int)sched_cars->size() << endl;
			for (const Car_in_database& new_car : scheduling_cars) {
				cout << new_car.id << " | " << new_car.OT << " | " << new_car.position << " | " << (int)new_car.length << " | " << (int)new_car.lane << " | " << new_car.current_turn << endl;
			}
			cout << "------------------  " << endl;
			for (const auto& [car_id, old_car] : *sched_cars) {
				cout << car_id << " | " << old_car.OT << " | " << old_car.position << " | " << (int)old_car.length << " | " << (int)old_car.lane << " | " << old_car.D << " | " << old_car.current_turn << endl;
			}
		}
	}
	*/

	// part 8: solve the problem
	const MPSolver::ResultStatus result_status = solver->Solve();

	// Check that the problem has an optimal solution.
	if (result_status != MPSolver::FEASIBLE && result_status != MPSolver::OPTIMAL) {
		cout << "The problem has no solution!" << endl;

		cout << "==============" << endl;
		for (auto& car : scheduling_cars) {
			cout << car.id << " | " << car.position << " | " << (int)car.length << " | " << car.OT << " ||| ";
		}
		cout << endl;
		for (const auto& [car_id, old_car] : *sched_cars) {
			cout << car_id << " | " << old_car.position << " | " << (int)old_car.length << " | " << old_car.OT << " ||| ";
		}
		cout << endl;
	}

	// Update the delays
	target_car.D = D_solver_variables[target_car.id]->solution_value();
}