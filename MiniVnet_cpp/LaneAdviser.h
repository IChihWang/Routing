#include "globalConst.h"
#include <vector>
#include <map>
#include <string>

#ifndef COORD
#define COORD
typedef tuple<int, int> Coord;
#endif

// Defined in main.cpp
extern map<string, vector< map<char, uint8_t> >> lane_dict;
extern map<string, double> inter_length_dict;

typedef tuple<uint8_t, char> Trajectory_ID;

class Lane_Adviser {
public:
	Lane_Adviser();

	void advise_lane(const Car& car);
	void update_Table_from_cars(const map<string, Car_in_database>& advising_car, const map<string, Car_in_database>& scheduling_cars, const map<string, Car_in_database>& scheduled_cars);
private:
	double timeMatrix[LANE_ADV_NUM][LANE_ADV_NUM] = {};
	map<Trajectory_ID, uint16_t> count_advised_not_secheduled_car_num;

	void update_Table(const Car_in_database& car, double time);
};