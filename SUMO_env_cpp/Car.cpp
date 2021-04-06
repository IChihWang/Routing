#include "Car.h"

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