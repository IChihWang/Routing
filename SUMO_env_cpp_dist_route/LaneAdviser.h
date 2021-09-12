#pragma once

#include <vector>
#include <map>
#include <string>
#include "global.h"
#include "Car.h"

#define DEFAULT_DATA_LENGTH 10
#define LANE_NUM_PER_DIRECTION 3
#define LANE_ADV_RESOLUTION 2
#define LANE_ADV_NUM (LANE_ADV_RESOLUTION*(LANE_NUM_PER_DIRECTION * 2 + 1))

// Defined in main.cpp

typedef tuple<uint8_t, char> Trajectory_ID;

class Lane_Adviser {
public:
	Lane_Adviser();

	uint8_t advise_lane(const Car& car, const bool spillback_lane_advise_avoid[]);
	void update_Table_from_cars(const map<string, Car*>& n_sched_car, const map<string, Car*>& advised_n_sched_car);
private:
	double timeMatrix[LANE_ADV_NUM][LANE_ADV_NUM] = {};
	map<Trajectory_ID, uint16_t> count_advised_not_secheduled_car_num;
	map<Trajectory_ID, uint8_t> all_default_lane;

	void update_Table(const Car& car, double time);
	double get_Max_Time(uint8_t lane, char turn, double time_matrix[][LANE_ADV_NUM]);
	void update_Table_After_Advise(const uint8_t& lane, const char& turn, const uint8_t& car_length, double time_matrix[][LANE_ADV_NUM]);
	void reset_Table();
};