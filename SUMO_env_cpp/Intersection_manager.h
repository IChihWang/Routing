#pragma once
#include <tuple>
#include <map>
#include "Car.h"
using namespace std;

#ifndef COORD
#define COORD
typedef tuple<uint16_t, uint16_t> Coord;
#endif

class IntersectionManager {
public:
	Coord id;

	// Break down to these to reduce the search space
	map<string, Car> car_list;		// Cars that needs to be handled
	map<string, Car> az_list;
	map<string, Car> pz_list;
	map<string, Car> cc_list;		// Cars under Cruse Control in CCZ
	map<string, Car> leaving_cars;	// Cars just entered the intersection (leave the CC zone)

	double schedule_period_count = 0;

	IntersectionManager(){}
	IntersectionManager(Coord id): id(id){}
};