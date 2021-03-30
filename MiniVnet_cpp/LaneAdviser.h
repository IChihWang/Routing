#pragma once

#include "globalConst.h"
#include <vector>
#include <map>
#include <string>

// Defined in main.cpp
extern map<string, vector< map<char, uint8_t> >> lane_dict;
extern map<string, double> inter_length_dict;

typedef tuple<uint8_t, char> Trajectory_ID;

class Lane_Adviser {
public:
	Lane_Adviser();

	uint8_t advise_lane(const Car& car, const bool spillback_lane_advise_avoid[]);
	void update_Table_from_cars(const map<string, Car_in_database>& advising_car, const map<string, Car_in_database>& scheduling_cars, const map<string, Car_in_database>& scheduled_cars);
private:
	double timeMatrix[LANE_ADV_NUM][LANE_ADV_NUM] = {};
	map<Trajectory_ID, uint16_t> count_advised_not_secheduled_car_num;
	map<Trajectory_ID, uint8_t> all_default_lane;

	void update_Table(const Car_in_database& car, double time);
	double get_Max_Time(uint8_t lane, char turn, double time_matrix[][LANE_ADV_NUM]);
	void update_Table_After_Advise(const uint8_t& lane, const char& turn, const uint8_t& car_length, double time_matrix[][LANE_ADV_NUM]);
};