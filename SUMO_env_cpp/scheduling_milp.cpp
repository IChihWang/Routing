#include "ortools/linear_solver/linear_solver.h"
#include <algorithm>
#include <iostream>
#include "global.h"
#include "intersection_manager.h"
#include "Car.h"

using namespace std;
using namespace operations_research;

void IntersectionManager::scheduling(map<string, Car*>& sched_car, map<string, Car*>& n_sched_car, map<string, Car*>& advised_n_sched_car) {

	Roadrunner_P(sched_car, n_sched_car, advised_n_sched_car);
	lane_advisor.update_Table_from_cars(n_sched_car, advised_n_sched_car);

	for (auto& [car_id, car_ptr] : n_sched_car) {
		car_ptr->zone_state = "scheduled";
		car_ptr->is_scheduled = true;
	}

	#ifdef PEDESTRIAN
	// Update the pedestrian ime list
	for (uint8_t direction = 0; direction < 4; direction++) {
		if (pedestrian_time_mark_list[direction] > 0)
			pedestrian_time_mark_list[direction] -= schedule_period_count;
		if (pedestrian_time_mark_list[direction] < -PEDESTRIAN_TIME_GAP)
			pedestrian_time_mark_list[direction] = 0;
	}
	#endif
}

void IntersectionManager::Roadrunner_P(map<string, Car*>& old_cars, map<string, Car*>& new_cars, map<string, Car*>& advised_n_sched_car) {
	// cout << "========================================" << endl;
	for (auto& [car_id, car_ptr] : new_cars) {
		double OT = car_ptr->position / V_MAX;
		car_ptr->OT = OT + SUMO_TIME_ERR;
	}

	// part 1: build the solver
	unique_ptr<MPSolver> solver(MPSolver::CreateSolver("SCIP"));
	const double infinity = solver->infinity();

	// part 2: claim variables
	map<string, const MPVariable*> D_solver_variables;

	// part 3: claim parameters
	uint16_t pre_accumulate_car_len_lane[4 * LANE_NUM_PER_DIRECTION] = {};
	fill_n(pre_accumulate_car_len_lane, 4 * LANE_NUM_PER_DIRECTION, 2*CAR_MAX_LEN + HEADWAY);

	// Compute accumulated car len
	for (const auto& [car_id, old_car_ptr] : old_cars) {
		Car& old_car = *old_car_ptr;
		pre_accumulate_car_len_lane[old_car.dst_lane] += (old_car.length + HEADWAY);

		/*
		if (old_car.dst_lane != old_car.dst_lane_changed_to){
			int8_t for_step = 0;
			if (old_car.dst_lane_changed_to > old_car.dst_lane)
				for_step = -1;
			else if (old_car.dst_lane_changed_to < old_car.dst_lane)
				for_step = 1;

			for (uint8_t other_lane_idx = old_car.dst_lane_changed_to; other_lane_idx != old_car.dst_lane; other_lane_idx += for_step) {
				pre_accumulate_car_len_lane[old_car.dst_lane_changed_to] += (old_car.length + HEADWAY);
			}
		}
		*/
		
		
		//pre_accumulate_car_len_lane[old_car.dst_lane_changed_to] += (old_car.length + HEADWAY);

		
		if (old_car.dst_lane != old_car.dst_lane_changed_to)
			pre_accumulate_car_len_lane[old_car.dst_lane_changed_to] += (old_car.length + HEADWAY);
		

	}

	vector<pair<string, Car*>> sorted_scheduling_cars_list;
	for (auto& [car_id, car_ptr] : new_cars) {
		sorted_scheduling_cars_list.push_back(make_pair(car_id, car_ptr));
	}

	sort(sorted_scheduling_cars_list.begin(), sorted_scheduling_cars_list.end(), [](pair<string, Car*> a, pair<string, Car*> b) -> bool {return a.second->position < b.second->position; });
	double head_of_line_blocking_position[4 * LANE_NUM_PER_DIRECTION];
	fill_n(head_of_line_blocking_position, 4 * LANE_NUM_PER_DIRECTION, UINT16_MAX);
	double accumulate_car_len[4 * LANE_NUM_PER_DIRECTION];
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
				car.is_spillback_strict = true;
				car.is_spillback = true;
				new_cars.erase(car_id);    // Blocked by the car at the front
				continue;
			}

			accumulate_car_len[dst_lane_idx] += (double(car.length) + HEADWAY);
			double spillback_delay = 0;

			if (accumulate_car_len[dst_lane_idx] > 0) {
				const vector<Car_Delay_Position_Record>& dst_car_delay_position = (others_road_info[dst_lane_idx])->car_delay_position;
				cout << "aaa " << car.id << " position: " << car.position << " lane: " << (int)car.dst_lane << endl;
				car.is_spillback = true;
				if (dst_car_delay_position.size() < 1 || (double(accumulate_car_len[dst_lane_idx])+double(CAR_MAX_LEN) + HEADWAY > dst_car_delay_position.back().position)) {
					// Skip because no records is found
					car.is_spillback_strict = true;
				}
				else {
					// Find the position in the list to compare
					uint32_t compare_dst_car_idx = 0;
					for (uint32_t dst_car_idx = 0; dst_car_idx < (uint32_t)dst_car_delay_position.size(); dst_car_idx++) {

						cout << "bbb " << dst_car_delay_position[dst_car_idx].car_id << " position: " << dst_car_delay_position[dst_car_idx].position << " delay: " << dst_car_delay_position[dst_car_idx].delay << endl;
						if (accumulate_car_len[dst_lane_idx] < dst_car_delay_position[dst_car_idx].position) {
							compare_dst_car_idx = dst_car_idx;
							break;
						}
					}

					spillback_delay = dst_car_delay_position[compare_dst_car_idx].delay;
					cout << "bbb-1 " << " accumulate_car_len: " << accumulate_car_len[dst_lane_idx] << endl;
				}

				spillback_delay_record[dst_lane_idx] = recorded_delay[dst_lane_idx];
			}
			else {
				spillback_delay_record[dst_lane_idx] = 0;
			}

			if (dst_lane_changed_to_idx != dst_lane_idx) {
				int8_t for_step = 0;
				if (dst_lane_changed_to_idx > dst_lane_idx)
					for_step = -1;
				else if (dst_lane_changed_to_idx < dst_lane_idx)
					for_step = 1;

				for (uint8_t other_lane_idx = dst_lane_changed_to_idx; other_lane_idx != dst_lane_idx; other_lane_idx += for_step) {

					if (other_lane_idx != dst_lane_idx) {
						accumulate_car_len[other_lane_idx] += (car.length + HEADWAY);
						double spillback_delay_dst_lane_changed_to = 0;
						const vector<Car_Delay_Position_Record>& dst_car_delay_position = (others_road_info[other_lane_idx])->car_delay_position;

						if (accumulate_car_len[other_lane_idx] > 0) {
							car.is_spillback = true;

							if (dst_car_delay_position.size() < 1 || (double(accumulate_car_len[other_lane_idx]) + double(CAR_MAX_LEN) + HEADWAY > dst_car_delay_position.back().position)) {
								// Skip because no records is found
								car.is_spillback_strict = true;
							}
							else {
								// Find the position in the list to compare
								uint32_t compare_dst_car_idx = 0;
								for (uint32_t dst_car_idx = 0; dst_car_idx < (uint32_t)dst_car_delay_position.size(); dst_car_idx++) {
									cout << "ccc lane:" << (int)other_lane_idx << " " << dst_car_delay_position[dst_car_idx].car_id << " position: " << dst_car_delay_position[dst_car_idx].position << " delay: " << dst_car_delay_position[dst_car_idx].delay << endl;
									if (accumulate_car_len[other_lane_idx] < dst_car_delay_position[dst_car_idx].position) {
										compare_dst_car_idx = dst_car_idx;
										break;
									}
								}

								double spillback_delay_alter = dst_car_delay_position[compare_dst_car_idx].delay;
								spillback_delay = max(spillback_delay, spillback_delay_alter);
								cout << "ccc-1 " << " accumulate_car_len: " << accumulate_car_len[other_lane_idx] << endl;
							}
							spillback_delay_record[other_lane_idx] = recorded_delay[other_lane_idx];
						}
						else {
							spillback_delay_record[other_lane_idx] = 0;
						}
					}
				}
			}

			cout << "ddd " << car.id << " spillback_delay: " << spillback_delay << endl;
			if (car.id == "car_2424" && car.is_spillback_strict == false) {
				//system("Pause");
			}

			if (car.is_spillback_strict == true) {
				// remove car from scheduling_cars
				new_cars.erase(car_id);

				if (car.position < head_of_line_blocking_position[lane_idx]) {
					head_of_line_blocking_position[lane_idx] = car.position;
				}
			}
			else {
				double min_d_add = 0;
				if (car.position < CCZ_LEN) {
					min_d_add = (2 * 2 * CCZ_ACC_LEN / (V_MAX + 0)) - (2 * CCZ_ACC_LEN / V_MAX);	 // Cost of fully stop
					double add_blind_car_delay = (LANE_WIDTH * LANE_NUM_PER_DIRECTION * 2) - (car.position / V_MAX + min_d_add);
					add_blind_car_delay = max(0.0, add_blind_car_delay);
					min_d_add += add_blind_car_delay;
				}

				if (car.current_turn == 'S') {
					MPVariable* const temp_D = solver->MakeNumVar(max(0 + min_d_add, spillback_delay), infinity, "d" + car.id);
					D_solver_variables[car.id] = temp_D;
				}
				else {
					double min_d = (2.0 * CCZ_DEC2_LEN / (double(V_MAX) + TURN_SPEED)) - (double(CCZ_DEC2_LEN) / V_MAX);
					MPVariable* const temp_D = solver->MakeNumVar(max(min_d + min_d_add, spillback_delay), infinity, "d" + car.id);
					D_solver_variables[car.id] = temp_D;
				}
			}
		}
		else {
			if (car.position > head_of_line_blocking_position[lane_idx]) {
				new_cars.erase(car_id);	  //Blocked by the car at the front
			}
			else {
				if (car.current_turn == 'S') {
					MPVariable* const temp_D = solver->MakeNumVar(0, infinity, "d" + car.id);
					D_solver_variables[car.id] = temp_D;
				}
				else {
					double min_d = (2.0 * CCZ_DEC2_LEN / (double(V_MAX) + TURN_SPEED)) - (double(CCZ_DEC2_LEN) / V_MAX);
					MPVariable* const temp_D = solver->MakeNumVar(min_d, infinity, "d" + car.id);
					D_solver_variables[car.id] = temp_D;
				}
			}
		}
	}

	// part 4: set constrain (10) (Car on same lane, rear-end collision avoidance)
	// (1) old car and new car
	for (const auto& [n_car_id, new_car_ptr] : new_cars) {
		for (const auto& [o_car_id, old_car_ptr] : old_cars) {
			Car& new_car = *new_car_ptr;
			Car& old_car = *old_car_ptr;

			if (new_car.lane == old_car.lane) {
				double bound = old_car.length / old_car.speed_in_intersection + (old_car.OT + old_car.D);
				bound += HEADWAY / old_car.speed_in_intersection;
				if (new_car.current_turn == 'S' && old_car.current_turn != 'S') {
					bound += (double(V_MAX) - TURN_SPEED) * (CCZ_DEC2_LEN) / (V_MAX * (double(V_MAX) + TURN_SPEED));
				}
				bound = bound - new_car.OT;
				MPConstraint* const tmp_conts = solver->MakeRowConstraint(bound, infinity);
				tmp_conts->SetCoefficient(D_solver_variables[new_car.id], 1);
				// cout << "4-1.    " << n_car_id << " " << o_car_id << " " << bound << endl;
			}
		}
	}

	// (2) two new cars
	vector<Car*> new_cars_vector;
	for (auto& [car_id, car_ptr] : new_cars) {
		new_cars_vector.push_back(car_ptr);
	}
	for (uint32_t i = 0; i < new_cars_vector.size(); i++) {
		for (uint32_t j = i+1; j < new_cars_vector.size(); j++) {
			Car* car_a_ptr = new_cars_vector[i];
			Car* car_b_ptr = new_cars_vector[j];

			if (car_a_ptr->lane == car_b_ptr->lane) {

				// Ensure car_a_ptr->OT > car_b_ptr->OT
				if (car_a_ptr->OT < car_b_ptr->OT) {
					swap(car_a_ptr, car_b_ptr);
				}

				double bound = car_b_ptr->length / car_b_ptr->speed_in_intersection - car_a_ptr->OT + car_b_ptr->OT;
				bound += HEADWAY / car_b_ptr->speed_in_intersection;
				if (car_a_ptr->current_turn == 'S' && car_b_ptr->current_turn != 'S') {
					bound += (double(V_MAX) - TURN_SPEED) * (CCZ_DEC2_LEN) / (double(V_MAX) * (double(V_MAX) + TURN_SPEED));
				}

				// cout << "4-2.    " << car_a_ptr->id << " " << car_b_ptr->id << " " << bound << endl;
				MPConstraint* const tmp_conts = solver->MakeRowConstraint(bound, infinity);
				tmp_conts->SetCoefficient(D_solver_variables[car_a_ptr->id], 1);
				tmp_conts->SetCoefficient(D_solver_variables[car_b_ptr->id], -1);
			}
		}
	}

	// part 5: set constrain (11) (two new cars in the intersection)
	for (uint32_t i = 0; i < new_cars_vector.size(); i++) {
		for (uint32_t j = i + 1; j < new_cars_vector.size(); j++) {
			Car& car_i = *(new_cars_vector[i]);
			Car& car_j = *(new_cars_vector[j]);

			if (car_i.lane == car_j.lane) {
				continue;
			}

			tuple<double, double> conflict_region_data = get_Conflict_Region(car_i, car_j);
			if (conflict_region_data != tuple<double, double>(0, 0)) {
				double tau_S1_S2 = get<0>(conflict_region_data);
				double tau_S2_S1 = get<1>(conflict_region_data);

				MPVariable* const flag = solver->MakeIntVar(0, 1, string("flag_new_new_") + car_i.id + "_" + car_j.id);

				double bound_2 = -car_j.OT + car_i.OT + tau_S1_S2 - LARGE_NUM;
				// cout << "5-1.    " << car_i.id << " " << car_j.id << " " << bound_2 << " " << tau_S1_S2 << endl;
				MPConstraint* const tmp_conts2 = solver->MakeRowConstraint(bound_2, infinity);
				tmp_conts2->SetCoefficient(D_solver_variables[car_i.id], -1);
				tmp_conts2->SetCoefficient(D_solver_variables[car_j.id], 1);
				tmp_conts2->SetCoefficient(flag, -LARGE_NUM);

				double bound_1 = -car_i.OT + car_j.OT + tau_S2_S1;
				// cout << "5-2.    " << car_i.id << " " << car_j.id << " " << bound_1 << " " << tau_S2_S1 << endl;
				MPConstraint* const tmp_conts1 = solver->MakeRowConstraint(bound_1, infinity);
				tmp_conts1->SetCoefficient(D_solver_variables[car_i.id], 1);
				tmp_conts1->SetCoefficient(D_solver_variables[car_j.id], -1);
				tmp_conts1->SetCoefficient(flag, LARGE_NUM);
			}
		}
	}

	// part 6: set constrain (12) (one new car and one old car in the intersection)
	for (const auto& [n_car_id, new_car_ptr] : new_cars) {
		for (const auto& [o_car_id, old_car_ptr] : old_cars) {
			Car& new_car = *new_car_ptr;
			Car& old_car = *old_car_ptr;

			if (new_car.lane == old_car.lane) {
				continue;
			}

			tuple<double, double> conflict_region_data = get_Conflict_Region(new_car, old_car);
			if (conflict_region_data != tuple<double, double>(0, 0)) {
				double tau_S1_S2 = get<0>(conflict_region_data);
				double tau_S2_S1 = get<1>(conflict_region_data);

				MPVariable* const flag = solver->MakeIntVar(0, 1, string("flag_old_new_") + old_car.id + "_" + new_car.id);

				double bound_3 = -old_car.D - old_car.OT + new_car.OT + tau_S1_S2 - LARGE_NUM;
				// cout << "6-1.    " << n_car_id << " " << o_car_id << " " << bound_3 << " " << tau_S1_S2 << endl;
				MPConstraint* const tmp_conts3 = solver->MakeRowConstraint(bound_3, infinity);
				tmp_conts3->SetCoefficient(D_solver_variables[new_car.id], -1);
				tmp_conts3->SetCoefficient(flag, -LARGE_NUM);

				double bound_4 = old_car.D + old_car.OT - new_car.OT + tau_S2_S1;
				// cout << "6-2.    " << n_car_id << " " << o_car_id << " " << bound_4 << " " << tau_S2_S1 << endl;
				MPConstraint* const tmp_conts4 = solver->MakeRowConstraint(bound_4, infinity);
				tmp_conts4->SetCoefficient(D_solver_variables[new_car.id], 1);
				tmp_conts4->SetCoefficient(flag, LARGE_NUM);
			}
		}
	}

	#ifdef PEDESTRIAN
	// part 7: pedestrian
	for (const auto& [car_id, car_ptr] : new_cars) {
		Car& car = *car_ptr;
		uint8_t in_dir = car.in_direction;
		uint8_t out_dir = car.out_direction;

		// Set pedestrian constraint if "in" is not none
		if (pedestrian_time_mark_list[in_dir] > 0) {
			double avoid_start_AT = pedestrian_time_mark_list[in_dir] - car.length / car.speed_in_intersection;
			double avoid_end_AT = pedestrian_time_mark_list[in_dir] + PEDESTRIAN_TIME_GAP;

			if (avoid_start_AT - car.OT <= 0) {
				double bound_p1 = avoid_end_AT - car.OT;
				MPConstraint* const tmp_conts1 = solver->MakeRowConstraint(bound_p1, solver->infinity());
				tmp_conts1->SetCoefficient(D_solver_variables[car_id], 1);
			}
			else {
				MPVariable* const flag = solver->MakeIntVar(0, 1, string("flag_ped_in_") + car_id);

				double bound_p2 = avoid_end_AT - car.OT - LARGE_NUM;
				MPConstraint* const tmp_conts2 = solver->MakeRowConstraint(bound_p2, infinity);
				tmp_conts2->SetCoefficient(D_solver_variables[car_id], 1);
				tmp_conts2->SetCoefficient(flag, -LARGE_NUM);

				double bound_p3 = -(avoid_start_AT - car.OT);
				MPConstraint* const tmp_conts3 = solver->MakeRowConstraint(bound_p3, infinity);
				tmp_conts3->SetCoefficient(D_solver_variables[car_id], -1);
				tmp_conts3->SetCoefficient(flag, LARGE_NUM);
			}
		}

		// Set pedestrian constraint if "out" is not none
		if (pedestrian_time_mark_list[out_dir] > 0) {
			double avoid_start_AT = pedestrian_time_mark_list[out_dir] - car.length / V_MAX;
			double avoid_end_AT = pedestrian_time_mark_list[out_dir] + PEDESTRIAN_TIME_GAP;
			double travel_in_inter_time = get_Intertime(car.lane, car.current_turn);

			if (avoid_start_AT - car.OT - travel_in_inter_time <= 0) {
				double bound = avoid_end_AT - car.OT - travel_in_inter_time;
				MPConstraint* const tmp_conts1 = solver->MakeRowConstraint(bound, infinity);
				tmp_conts1->SetCoefficient(D_solver_variables[car_id], 1);
			}
			else {
				MPVariable* const flag = solver->MakeIntVar(0, 1, string("flag_ped_out_") + car_id);

				double bound_p2 = avoid_end_AT - car.OT - travel_in_inter_time - LARGE_NUM;
				MPConstraint* const tmp_conts2 = solver->MakeRowConstraint(bound_p2, infinity);
				tmp_conts2->SetCoefficient(D_solver_variables[car_id], 1);
				tmp_conts2->SetCoefficient(flag, -LARGE_NUM);

				double bound_p3 = -(avoid_start_AT - car.OT - travel_in_inter_time);
				MPConstraint* const tmp_conts3 = solver->MakeRowConstraint(bound_p3, infinity);
				tmp_conts3->SetCoefficient(D_solver_variables[car_id], -1);
				tmp_conts3->SetCoefficient(flag, LARGE_NUM);
			}
		}
	}
	#endif


	// part 7: set objective
	MPObjective* const objective = solver->MutableObjective();
	for (const auto& [car_id, new_car_ptr] : new_cars) {
		objective->SetCoefficient(D_solver_variables[car_id], 1);
	}
	objective->SetMinimization();

	// part 8: solve the problem
	const MPSolver::ResultStatus result_status = solver->Solve();

	// Check that the problem has an optimal solution.
	if (result_status != MPSolver::FEASIBLE && result_status != MPSolver::OPTIMAL) {
		 cout << "The problem has no solution!" << endl;
	}

	// Update the delays
	for (const auto& [car_id, D_variable] : D_solver_variables) {
		new_cars[car_id]->D = D_solver_variables[car_id]->solution_value();
		new_cars[car_id]->is_scheduled = true;
	}
}