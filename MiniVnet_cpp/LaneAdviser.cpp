#include <numeric>	//iota
#include <algorithm>	//sort
#include "global.h"

Lane_Adviser::Lane_Adviser() {
	for (uint8_t lane_idx = 0; lane_idx < LANE_NUM_PER_DIRECTION; lane_idx++) {
		count_advised_not_secheduled_car_num[Trajectory_ID(lane_idx, 'L')] = 0;
		count_advised_not_secheduled_car_num[Trajectory_ID(lane_idx, 'R')] = 0;
		count_advised_not_secheduled_car_num[Trajectory_ID(lane_idx, 'S')] = 0;
	}

	for (uint8_t i = 0; i < 4; i++) {
		all_default_lane[Trajectory_ID(i, 'L')] = i * LANE_NUM_PER_DIRECTION;
		all_default_lane[Trajectory_ID(i, 'R')] = i * LANE_NUM_PER_DIRECTION + (LANE_NUM_PER_DIRECTION - 1);
		all_default_lane[Trajectory_ID(i, 'S')] = (all_default_lane[Trajectory_ID(i, 'L')] + all_default_lane[Trajectory_ID(i, 'R')]) / 2;
	}
}

uint8_t Lane_Adviser::advise_lane(const Car& car, const bool spillback_lane_advise_avoid[]) {
	uint8_t advise_lane = 0;


	// If spillback detected, avoid the other lanes
	if (spillback_lane_advise_avoid[car.lane] == true) {
		uint8_t start_lane = (car.lane / LANE_NUM_PER_DIRECTION) * LANE_NUM_PER_DIRECTION;
		uint8_t ideal_lane = start_lane;
		if (car.current_turn == 'R') {
			ideal_lane = start_lane + LANE_NUM_PER_DIRECTION - 1;
		}
		else if (car.current_turn == 'L') {
			ideal_lane = start_lane;
		}
		else {
			ideal_lane = start_lane + (LANE_NUM_PER_DIRECTION/2);
		}
		advise_lane = ideal_lane;
		return advise_lane;
	}

	// Sort out the LOTs and list the candidates
	uint8_t start_lane = (car.lane / LANE_NUM_PER_DIRECTION) * LANE_NUM_PER_DIRECTION;
	vector<double> occup_time_list;
	for (uint8_t idx = 0; idx < LANE_NUM_PER_DIRECTION; idx++) {
		occup_time_list.push_back(get_Max_Time(start_lane + idx, car.current_turn, timeMatrix) + get_Intertime(start_lane + idx, car.current_turn));
	}
	// Sort occup_time_list
	vector<uint8_t> candidate_list(occup_time_list.size());
	iota(candidate_list.begin(), candidate_list.end(), 0);
	sort(candidate_list.begin(), candidate_list.end(), [&occup_time_list](double a, double b) {return a < b; });

	// Get the shortest or the most ideal lane
	uint8_t ideal_lane = start_lane;
	if (car.current_turn == 'R') {
		ideal_lane = start_lane + LANE_NUM_PER_DIRECTION - 1;
	}
	else if (car.current_turn == 'L') {
		ideal_lane = start_lane;
	}
	else {
		// Find one mid-lane with smallest LOTs
		if (LANE_NUM_PER_DIRECTION > 2) {
			ideal_lane = start_lane + candidate_list[0];
		}
		else {
			for (uint8_t& lane_idx: candidate_list) {
				if (lane_idx != 0 && lane_idx != LANE_NUM_PER_DIRECTION) {
					ideal_lane = start_lane + lane_idx;
					break;
				}
			}
		}
	}
	advise_lane = ideal_lane;


	// Scan through the candidates and see if we want to change our candidates
	

	double ideal_timeMatrix[LANE_ADV_NUM][LANE_ADV_NUM];
	copy(&timeMatrix[0][0], &timeMatrix[0][0] + LANE_ADV_NUM * LANE_ADV_NUM, &ideal_timeMatrix[0][0]);
	update_Table_After_Advise(ideal_lane, car.current_turn, car.length, ideal_timeMatrix);

	// Recored the costs of other trajectories if ideal lane is chosen
	map<Trajectory_ID, double> ideal_others_LOT_list;
	for (const pair<Trajectory_ID, uint8_t>& data : all_default_lane) {
		Trajectory_ID key = data.first;
		uint8_t lane = data.second;
		char turn = get<1>(key);
		ideal_others_LOT_list[key] = get_Max_Time(lane, turn, ideal_timeMatrix);
	}

	for (const uint8_t& lane_i : candidate_list) {
		uint8_t candidate_lane = start_lane + lane_i;

		// The ideal lane has the smallest cost so far
		if (candidate_lane == ideal_lane) {
			break;
		}

		// Get the LOT costs if the candidate lane is chosen
		double candidate_timeMatrix[LANE_ADV_NUM][LANE_ADV_NUM];
		copy(&timeMatrix[0][0], &timeMatrix[0][0] + LANE_ADV_NUM * LANE_ADV_NUM, &candidate_timeMatrix[0][0]);
		update_Table_After_Advise(candidate_lane, car.current_turn, car.length, candidate_timeMatrix);

		map<Trajectory_ID, double> candidate_others_LOT_list;
		for (const pair<Trajectory_ID, uint8_t>& data : all_default_lane) {
			Trajectory_ID key = data.first;
			uint8_t lane = data.second;
			char turn = get<1>(key);
			candidate_others_LOT_list[key] = get_Max_Time(lane, turn, candidate_timeMatrix);
		}

		// Compare the LOTs
		bool is_better = true;
		for (const pair<Trajectory_ID, uint8_t>& data : all_default_lane) {
			Trajectory_ID key = data.first;
			uint8_t lane = data.second;
			char turn = get<1>(key);
			
			// Skip comparing the candidate lanes, because obviously it will increase
			if (lane == candidate_lane) {
				continue;
			}
			else if (count_advised_not_secheduled_car_num[key] > 0){
				if (candidate_others_LOT_list[key] > ideal_others_LOT_list[key]) {
					is_better = false;
					break;
				}

			}
		}

		if (!is_better) {
			continue;
		}
		else {
			advise_lane = candidate_lane;
			break;
		}
	}

	
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
	uint8_t direction = car.lane / LANE_NUM_PER_DIRECTION;
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

double Lane_Adviser::get_Max_Time(uint8_t lane, char turn, double time_matrix[][LANE_ADV_NUM]) {
	uint8_t direction = lane / LANE_NUM_PER_DIRECTION;
	string lookup_key = to_string(lane % LANE_NUM_PER_DIRECTION) + turn;

	double max_time = 0;
	if (direction == 0) {
		for (map<char, uint8_t> occupied_boxes : lane_dict[lookup_key]) {
			if (max_time < time_matrix[occupied_boxes['X']][occupied_boxes['Y']]) {
				max_time = time_matrix[occupied_boxes['X']][occupied_boxes['Y']];
			}
		}
	}
	else if (direction == 1) {
		for (map<char, uint8_t> occupied_boxes : lane_dict[lookup_key]) {
			if (max_time < time_matrix[LANE_ADV_NUM - 1 - occupied_boxes['Y']][occupied_boxes['X']]) {
				max_time = time_matrix[LANE_ADV_NUM - 1 - occupied_boxes['Y']][occupied_boxes['X']];
			}
		}
	}
	else if (direction == 2) {
		for (map<char, uint8_t> occupied_boxes : lane_dict[lookup_key]) {
			if (max_time < time_matrix[LANE_ADV_NUM - 1 - occupied_boxes['X']][occupied_boxes['Y']]) {
				max_time = time_matrix[LANE_ADV_NUM - 1 - occupied_boxes['X']][occupied_boxes['Y']];
			}
		}
	}
	else if (direction == 3) {
		for (map<char, uint8_t> occupied_boxes : lane_dict[lookup_key]) {
			if (max_time < time_matrix[occupied_boxes['Y']][LANE_ADV_NUM - 1 - occupied_boxes['X']]) {
				max_time = time_matrix[occupied_boxes['Y']][LANE_ADV_NUM - 1 - occupied_boxes['X']];
			}
		}
	}

	return max_time;
}

void Lane_Adviser::update_Table_After_Advise(const uint8_t& lane, const char& turn, const uint8_t& car_length, double time_matrix[][LANE_ADV_NUM]) {
	uint8_t direction = lane / LANE_NUM_PER_DIRECTION;
	string lookup_key = to_string(lane % LANE_NUM_PER_DIRECTION) + turn;
	
	// Determine the speed
	double speed = 0;
	if (turn == 'S') {
		speed = _V_MAX;
	}
	else {
		speed = _TURN_SPEED;
	}

	if (direction == 0) {
		for (map<char, uint8_t> occupied_boxes : lane_dict[lookup_key]) {
			double time = get_Intertime(lane, turn) + car_length / speed;
			timeMatrix[occupied_boxes['X']][occupied_boxes['Y']] += time;
		}
	}
	else if (direction == 1) {
		for (map<char, uint8_t> occupied_boxes : lane_dict[lookup_key]) {
			double time = get_Intertime(lane, turn) + car_length / speed;
			timeMatrix[LANE_ADV_NUM - 1 - occupied_boxes['Y']][occupied_boxes['X']] += time;
		}
	}
	else if (direction == 2) {
		for (map<char, uint8_t> occupied_boxes : lane_dict[lookup_key]) {
			double time = get_Intertime(lane, turn) + car_length / speed;
			timeMatrix[LANE_ADV_NUM - 1 - occupied_boxes['X']][occupied_boxes['Y']] += time;
		}
	}
	else if (direction == 3) {
		for (map<char, uint8_t> occupied_boxes : lane_dict[lookup_key]) {
			double time = get_Intertime(lane, turn) + car_length / speed;
			timeMatrix[occupied_boxes['Y']][LANE_ADV_NUM - 1 - occupied_boxes['X']] += time;
		}
	}
}