#include "Car.h"
#include <cmath>
#include <algorithm>
using namespace std;

Car::Car(string car_id, uint8_t length, uint8_t lane, char turn, char next_turn): 
	id(car_id), length(length), original_lane(lane), lane(lane), desired_lane(lane),
	current_turn(turn), next_turn(next_turn) {

	set_turning(turn, next_turn);
}

void Car::set_turning(char turn, char next_turn) {
	in_direction = lane / LANE_NUM_PER_DIRECTION;

	if (current_turn == 'S') {
		out_direction = (in_direction + 2) % 4;
	}
	else if (current_turn == 'R') {
		out_direction = (in_direction + 1) % 4;
	}
	else if (current_turn == 'L') {
		out_direction = (in_direction + 4 - 1) % 4;	// +4 to make it positive
	}

	uint8_t out_sub_lane = (LANE_NUM_PER_DIRECTION - lane % LANE_NUM_PER_DIRECTION - 1);
	dst_lane = uint8_t(out_direction * LANE_NUM_PER_DIRECTION + out_sub_lane);     // Destination lane before next lane change

	if (current_turn == 'R') {
		out_sub_lane = 0;
	}
	else if (current_turn == 'L') {
		out_sub_lane = LANE_NUM_PER_DIRECTION - 1;
	}
	else if (current_turn == 'S') {
		out_sub_lane = uint8_t(LANE_NUM_PER_DIRECTION / 2);
	}

	dst_lane_changed_to = uint8_t(out_direction * LANE_NUM_PER_DIRECTION + out_sub_lane);  // Destination lane after next lane change

	// Determine the speed in the intersection
	if (current_turn == 'S') {
		speed_in_intersection = V_MAX;
	}
	else {
		speed_in_intersection = TURN_SPEED;
	}
}

void Car::handle_CC_behavior(map<string, Car*>& car_list) {
	Car* front_car_ptr = nullptr;
	double front_distance = 0;
	double front_speed = 0;
	CC_slowdown_timer -= _TIME_STEP;

	pair<string, double> leader_tuple = traci.vehicle.getLeader(id, 0);
	if (leader_tuple.first != "") {
		if (car_list.find(leader_tuple.first) != car_list.end()) {
			string front_car_ID = leader_tuple.first;
			front_car_ptr = car_list[front_car_ID];
			front_distance = leader_tuple.second + 3;    // Because SUMO measre the distance with a given Gap

			if (CC_front_pos_diff == 0) {
				CC_front_pos_diff = position - front_car_ptr->position;
			}
		}
	}
	CC_front_car = front_car_ptr;
	front_speed = CC_get_front_speed();
	
	// 1. Detect if the front car is too close
	if ((CC_state == "") || (!(CC_state.find("Platoon") != string::npos || CC_state.find("Entering") != string::npos))) {
		if (front_car_ptr != nullptr) {
			double my_speed = traci.vehicle.getSpeed(id) + traci.vehicle.getAcceleration(id);
			double min_catch_up_time = (my_speed - front_speed) / MAX_ACC;
			double min_distance = (my_speed - front_speed) * min_catch_up_time;

			if (min_catch_up_time > 0 && front_distance < min_distance + HEADWAY) {
				CC_state = "Platoon_catchup";
			}
		}
	}

	// 2. If the car is ready for stopping
	if ((position < (2 * CCZ_ACC_LEN + CCZ_DEC2_LEN)) && ((CC_state == "") || (!(CC_state.find("Entering") != string::npos)))) {
		CC_state = "Entering_decelerate";
		double slow_down_speed = 0;
		double my_speed = traci.vehicle.getSpeed(id);

		if (!is_scheduled)
			slow_down_speed = 0.001;
		else {
			// Compute the slowdown speed
			double T = OT + D - ((CCZ_DEC2_LEN) / ((speed_in_intersection + V_MAX) / 2));
			double max_total_time = (position - (CCZ_ACC_LEN + CCZ_DEC2_LEN)) / (my_speed / 2) + CCZ_ACC_LEN / (V_MAX / 2);

			if (T > max_total_time) {
				CC_is_stop_n_go = true;
				slow_down_speed = 0.001;
			}
			else if (T < 0) {
				slow_down_speed = V_MAX;
			}
			else {
				double x1 = position - (CCZ_ACC_LEN + CCZ_DEC2_LEN);
				double x2 = CCZ_ACC_LEN;
				double v1 = my_speed;
				double vm = V_MAX;

				double a = T;
				double b = (vm * T + v1 * T - 2 * x1 - 2 * x2);
				double c = (vm * v1 * T - 2 * x1 * vm - 2 * x2 * v1);

				slow_down_speed = max((-b - sqrt(pow(b,2) - 4 * a * c)) / (2 * a), (-b + sqrt(pow(b, 2) - 4 * a * c)) / (2 * a));
				slow_down_speed = min(slow_down_speed, V_MAX);
			}
		}
		
		CC_slow_speed = slow_down_speed;
		traci.vehicle.setMaxSpeed(id, slow_down_speed);
		double dec_time = (position - (CCZ_ACC_LEN + CCZ_DEC2_LEN)) / ((my_speed + slow_down_speed) / 2);
		CC_slowdown_timer = dec_time;
		traci.vehicle.slowDown(id, slow_down_speed, dec_time);
	}
	else if ((CC_state == "Entering_decelerate" || CC_state == "Entering_wait") && (CC_slowdown_timer <= 0)) {
		traci.vehicle.setSpeed(id, CC_slow_speed);

		double wait_time = 99999;   // inf and wait
		if (is_scheduled)
			wait_time = OT + D - ((position - CCZ_DEC2_LEN) / ((V_MAX + CC_slow_speed) / 2)) - ((CCZ_DEC2_LEN) / ((speed_in_intersection + V_MAX) / 2));

		if (wait_time > 0)
			CC_state = "Entering_wait";
		else {
			CC_state = "Entering_accerlerate";

			traci.vehicle.setMaxSpeed(id, V_MAX);
			double dec_time = 0;
			if (position > CCZ_DEC2_LEN)
				dec_time = (position - CCZ_DEC2_LEN) / ((CC_slow_speed + V_MAX) / 2);
			else
				dec_time = position / ((CC_slow_speed + V_MAX) / 2);
			CC_slowdown_timer = dec_time;
			traci.vehicle.slowDown(id, V_MAX, dec_time);
		}
	}
	else if ((CC_state == "Entering_accerlerate") && (CC_slowdown_timer <= 0)) {
		CC_state = "Entering_adjusting_speed";

		traci.vehicle.setMaxSpeed(id, speed_in_intersection);
		double dec_time = position / ((speed_in_intersection + V_MAX) / 2);
		CC_slowdown_timer = dec_time;
		traci.vehicle.slowDown(id, speed_in_intersection, dec_time);
	}
	else if ((CC_state == "Entering_adjusting_speed") && (CC_slowdown_timer <= 0)) {
		CC_state = "Entering_intersection";
		traci.vehicle.setSpeed(id, speed_in_intersection);
	}
	else if (CC_state == "Platoon_catchup") {
		if (front_car_ptr == nullptr) {
			double my_speed = traci.vehicle.getSpeed(id);
			double target_speed = min(V_MAX, my_speed + MAX_ACC * _TIME_STEP);
			traci.vehicle.setSpeed(id, target_speed);
			if (target_speed == V_MAX)
				CC_state = "Keep_V_MAX";
		}
		else {
			double my_speed = traci.vehicle.getSpeed(id);
			double min_catch_up_time = (my_speed - 0) / MAX_ACC;
			double min_distance = (my_speed - 0) * min_catch_up_time;

			if (front_distance < min_distance) {
				double target_speed = max(front_speed, my_speed - MAX_ACC * _TIME_STEP);

				if (front_distance <= HEADWAY) {
					CC_state = "Platoon_following";
					traci.vehicle.setSpeed(id, front_speed);
				}
				else {
					if (front_speed > target_speed) {
						target_speed = min(V_MAX, my_speed + MAX_ACC * _TIME_STEP);
						traci.vehicle.setSpeed(id, target_speed);
					}
					else {
						target_speed = max(front_speed + MAX_ACC * _TIME_STEP, my_speed - MAX_ACC * _TIME_STEP);
						traci.vehicle.setSpeed(id, target_speed);
					}
				}
			}
			else {
				double target_speed = min(V_MAX, my_speed + MAX_ACC * _TIME_STEP);
				traci.vehicle.setSpeed(id, target_speed);
			}
		}
	}
	else if (CC_state == "Platoon_following") {

		if (front_car_ptr == nullptr) {
			double my_speed = traci.vehicle.getSpeed(id);
			double target_speed = min(V_MAX, my_speed + MAX_ACC * _TIME_STEP);
			traci.vehicle.setSpeed(id, target_speed);
			if (target_speed == V_MAX)
				CC_state = "Keep_V_MAX";
		}
		else {
			if (front_distance > HEADWAY) {
				double my_speed = traci.vehicle.getSpeed(id);
				double target_speed = min(V_MAX, my_speed + MAX_ACC * _TIME_STEP);
				traci.vehicle.setSpeed(id, target_speed);
				CC_state = "Platoon_catchup";
			}
			else
				traci.vehicle.setSpeed(id, front_speed);
		}
	}
	else if (CC_state == "Preseting_start") {
		double my_speed = traci.vehicle.getSpeed(id);
		traci.vehicle.setMaxSpeed(id, V_MAX);
		double dec_time = (max(V_MAX - my_speed, my_speed - V_MAX)) / MAX_ACC;
		CC_slowdown_timer = dec_time;
		traci.vehicle.slowDown(id, V_MAX, dec_time);
		CC_state = "Preseting_done";
	}
	else if ((CC_state == "Preseting_done") && (CC_slowdown_timer <= 0)) {
		traci.vehicle.setSpeed(id, V_MAX);
	}
	else if (CC_state == "CruiseControl_ready") {
		if (CC_front_car != nullptr && CC_front_car->CC_shift == -1) {
			CC_front_car = nullptr;
		}

		CC_get_shifts(car_list);
		CC_get_slow_down_speed();
		if (CC_is_stop_n_go) {
			// Only stop at very closed to the intersection
			CC_state = "Keep_V_MAX";
		}
		else {
			CC_state = "CruiseControl_shift_start";
		}
	}
	else if ((CC_state == "CruiseControl_shift_start") && position < (CCZ_LEN - CC_shift)) {
		CC_state = "CruiseControl_decelerate";
		double speed = CC_slow_speed;

		// Delta : some small error that SUMO unsync with ideal case
		double delta = (CCZ_LEN - CC_shift) - position;
		double dec_time = (CCZ_ACC_LEN - delta) / ((V_MAX + speed) / 2);

		if (dec_time < 0)
			CC_state = "Keep_V_MAX";
		else {
			traci.vehicle.setMaxSpeed(id, speed);
			traci.vehicle.slowDown(id, speed, dec_time);
			CC_slowdown_timer = dec_time;
		}
	}
	else if ((CC_state == "CruiseControl_decelerate") && (CC_slowdown_timer <= 0)) {
		traci.vehicle.setSpeed(id, CC_slow_speed);
		CC_state = "CruiseControl_slowdown_speed";
	}
	else if ((CC_state == "CruiseControl_slowdown_speed") && (position <= (CCZ_ACC_LEN + CC_shift_end))) {
		CC_state = "CruiseControl_accelerate";
		// Delta : some small error that SUMO unsync with ideal case
		double delta = (CCZ_ACC_LEN + CC_shift_end) - position;
		double dec_time = (CCZ_ACC_LEN - delta) / ((V_MAX + CC_slow_speed) / 2);
		traci.vehicle.setMaxSpeed(id, V_MAX);
		traci.vehicle.slowDown(id, V_MAX, dec_time);
		CC_slowdown_timer = dec_time;
	}
	else if ((CC_state == "CruiseControl_accelerate") && (CC_slowdown_timer <= 0)) {
		CC_state == "CruiseControl_V_MAX";
		traci.vehicle.setSpeed(id, V_MAX);
	}

	return;
}

double Car::CC_get_front_speed() {
	if (CC_front_car != nullptr) {
		if (CC_front_car->CC_state != "" && CC_front_car->CC_state.find("Platoon") != string::npos)
			return CC_front_car->CC_get_front_speed();
		else
			return traci.vehicle.getSpeed(CC_front_car->id) + traci.vehicle.getAcceleration(CC_front_car->id) * _TIME_STEP;
	}
	else
		return traci.vehicle.getSpeed(id) + traci.vehicle.getAcceleration(id) * _TIME_STEP;
}

pair<double, double> Car::CC_get_shifts(map<string, Car*>& car_list) {
	// 1.1 Determine how much to advance the car acceleration (shift_end)
	bool is_catching_up_front = false;

	if (CC_front_car != nullptr && CC_front_car->CC_shift >= 0) {
		double shifting_end = CCZ_DEC2_LEN;
		double front_remain_D = (CC_front_car->OT + CC_front_car->D) - (CC_front_car->position / V_MAX);
		double catch_up_distance = (front_remain_D - D) * V_MAX;
		double diff_distance = position - CC_front_car->position;
		if ((diff_distance - catch_up_distance - CC_front_car->length) < HEADWAY) {
			// The car is going to catch up the front car
			shifting_end = CC_front_car->CC_shift_end + CC_front_car->length + HEADWAY;
			is_catching_up_front = true;
			CC_shift_end = shifting_end;
		}
	}

	// 1.2 Determine the upperbound of the delaying for a car to accelerate
	double cc_shift_max = CCZ_LEN - CC_shift_end - 2 * CCZ_ACC_LEN;
	if (is_catching_up_front && CC_front_car->CC_slow_speed < V_MAX) {
		// First, assume that the two cars decelerate at the same time(So the car has this shift)
		cc_shift_max = CC_front_car->CC_shift - CC_front_pos_diff;

		// The space between two cars(might < 0, because might smaller than HEADWAY)
		double space_between_two = max(CC_front_pos_diff - CC_front_car->length - HEADWAY, 0.0);

		// 2 Compute catch up time && reflect to the space
		double catch_up_t = space_between_two / (V_MAX - CC_front_car->CC_slow_speed);
		cc_shift_max += catch_up_t * V_MAX;
	}
	cc_shift_max = min(cc_shift_max, CCZ_LEN - CC_shift_end - 2 * CCZ_ACC_LEN);

	// 1.3 Determine the delay it desires. Reserving for the following cars
	// Count cars that'll enter CCZ during the delaying
	double reserve_shift = length + HEADWAY;
	double count_distance = D * V_MAX + length + HEADWAY;

	vector<Car*> count_car_list;
	for (const auto& [count_car_id, count_car_ptr] : car_list) {
		if ((count_car_ptr->lane == lane) && (count_car_ptr->position > position))
			count_car_list.push_back(count_car_ptr);
	}

	count_car_list.push_back(this);
	sort(count_car_list.begin(), count_car_list.end(), [](Car* a, Car* b) -> bool {return a->position < b->position; });
	double temp_D = D;

	for (uint32_t car_i = 1; car_i < count_car_list.size(); car_i++) {
		if (count_car_list[car_i]->position < count_car_list[car_i - 1]->position + count_distance) {
			reserve_shift += count_car_list[car_i]->length + HEADWAY;

			// If the car has been scheduled
			if (count_car_list[car_i]->is_scheduled) {
				count_distance = count_car_list[car_i]->D * V_MAX + count_car_list[car_i]->length + HEADWAY;
				temp_D = count_car_list[car_i]->D;
			}
			else
				count_distance = temp_D * V_MAX + count_car_list[car_i]->length + HEADWAY;
		}
		else
			break;
	}

	reserve_shift = min(reserve_shift, CCZ_LEN - CC_shift_end - 2 * CCZ_ACC_LEN);

	// 1.4 Decide the final shift
	double shifting = min(reserve_shift, cc_shift_max);
	CC_shift = shifting;

	return make_pair(shifting, CC_shift_end);
}

void Car::CC_get_slow_down_speed() {
	double AT = D + ((CCZ_LEN) / V_MAX) - (CC_shift / V_MAX) - ((CC_shift_end - CCZ_DEC2_LEN) / V_MAX) - (2 * CCZ_DEC2_LEN / (V_MAX + speed_in_intersection));

	double S = CCZ_ACC_LEN;
	double S2 = CCZ_LEN - 2 * CCZ_ACC_LEN - CC_shift - CC_shift_end;
	double T = ((CCZ_LEN) / V_MAX) + D - ((CC_shift + CC_shift_end - CCZ_DEC2_LEN) / V_MAX + 2 * CCZ_DEC2_LEN / (V_MAX + speed_in_intersection));

	double a = T;
	double b = -(S2 + 4 * S - T * V_MAX);
	double c = -S2 * V_MAX;

	double speed = max((-b - sqrt(pow(b, 2) - 4 * a * c)) / (2 * a), (-b + sqrt(pow(b, 2) - 4 * a * c)) / (2 * a));
	speed = min(speed, V_MAX);

	// Determine if there's stop && go
	if (speed < 1)
		CC_is_stop_n_go = true;

	CC_slow_speed = speed;
}