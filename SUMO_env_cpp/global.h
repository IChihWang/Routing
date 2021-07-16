#pragma once
#include "traci_lib/TraCIAPI.h"
#include "thread_worker.h"
#include <map>
#include <tuple>
using namespace std;

#define V_MAX 11.18
#define TURN_SPEED 10.0
#define LANE_NUM_PER_DIRECTION 3
#define SUMO_TIME_ERR 0

#define AZ_LEN 75.0
#define PZ_LEN 25.0
#define GZ_LEN 25.0
#define BZ_LEN 25.0
#define CCZ_LEN 50.0
#define TOTAL_LEN double(AZ_LEN+PZ_LEN+GZ_LEN+BZ_LEN+CCZ_LEN)

#define LANE_WIDTH 3.2
#define HEADWAY 3
#define CCZ_ACC_LEN 5.0
#define CCZ_DEC2_LEN 2.5
#define MAX_ACC double((V_MAX*V_MAX)/(2*CCZ_ACC_LEN))
#define CCZ_CATCHUP_MIN_SPEED 3

#define CAR_MAX_LEN 15
#define CAR_MIN_LEN 5
#define CAR_AVG_LEN ((CAR_MAX_LEN+CAR_MIN_LEN)/2)
#define DISTANCE 1.5

#define SCHEDULING_PERIOD double(GZ_LEN/V_MAX)
#define ROUTING_PERIOD_NUM (int((TOTAL_LEN/V_MAX/2.0)/SCHEDULING_PERIOD)-1)
#define ROUTING_PERIOD double(ROUTING_PERIOD_NUM*SCHEDULING_PERIOD)
#define LARGE_NUM 65535


#define PEDESTRIAN_TIME_GAP 0 // 60 second


extern map<string, vector< map<char, uint8_t> >> lane_dict;
extern map<string, double> inter_length_dict;
extern map<string, map<string, double> > inter_info;

class Car;
class IntersectionManager;

void read_load_adv_data();
void read_inter_info_data();
void read_inter_length_data();
tuple<double, double> get_Conflict_Region(Car car1, Car car2);
double get_Intertime(uint8_t lane, char turn);

class SUMO_Traci : public TraCIAPI {
public:
    SUMO_Traci() {};
    ~SUMO_Traci() {};
};

extern uint8_t _grid_size;
extern uint16_t _N_TIME_STEP;
extern float _TIME_STEP;
extern uint8_t _CHOOSE_CAR_OPTION;
extern uint8_t _TOP_N_CONGESTED;
extern uint8_t _THREAD_NUM;
extern uint8_t _ITERATION_NUM;
extern string _RANDOM_SEED;
extern string _ARRIVAL_RATE;
extern string _CAR_TIME_ERROR;


// Claim in main.cpp
extern SUMO_Traci traci;


// Defined in server.cpp
string get_string_from_double(double num, uint8_t percision);