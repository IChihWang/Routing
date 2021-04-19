#pragma once

#define DEFAULT_DATA_LENGTH 10
#define LANE_NUM_PER_DIRECTION 3
#define LANE_ADV_RESOLUTION 2
#define LANE_ADV_NUM (LANE_ADV_RESOLUTION*(LANE_NUM_PER_DIRECTION * 2 + 1))

#define DISTANCE 1.5		// 1.5 lane
#define LANE_WIDTH 3.2		// 3.2 meters

#define SCHEDULE_POSPONDED -1 // a flag
#define NOT_SCHEDULED -2 // a flag
#define SUMO_TIME_ERR 0
#define CAR_MAX_LEN 15
#define CCZ_ACC_LEN 5
#define CCZ_DEC2_LEN 2.5
#define LARGE_NUM 65535

// Defined in server.cpp
extern uint8_t _thread_num;
extern uint8_t _grid_size;
extern float _schedule_period;
extern uint8_t _routing_period_num;
extern float _GZ_BZ_CCZ_len;
extern uint8_t _HEADWAY;
extern float _V_MAX;
extern float _TURN_SPEED;
extern uint16_t _TOTAL_LEN;
extern float _routing_period;


#ifndef COORD
#define COORD
#include <tuple>
typedef std::tuple<int, int> Coord;
#endif