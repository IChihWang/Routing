#include <map>
#include <tuple>
#include <vector>
#include <shared_mutex>	// C++ 17
#include "element.h"
using namespace std;


// Defined in main.cpp

// Global variables
// Defined in element.cpp


// Defined in server.cpp
extern int _thread_num;
extern int _grid_size;
extern float _schedule_period;
extern int _routing_period_num;
extern float _GZ_BZ_CCZ_len;
extern float _HEADWAY;
extern float _V_MAX;
extern float _TURN_SPEED;
extern float _TOTAL_LEN;


// Defined in MiniVnet.cpp
#define DEFAULT_DATA_LENGTH 10
#define LANE_NUM_PER_DIRECTION 3


extern std::vector< std::map< std::tuple<int, int>, Intersection > > _database;
extern map<string, Car> _car_dict;

extern shared_mutex _database_g_mutex;
extern unique_lock<shared_mutex> _database_wLock;
extern shared_lock<shared_mutex> _database_rLock;

void create_grid_network();
void add_time_step();
void move_a_time_step();
void update_car(const string& car_id, const uint8_t& car_length, const string& src_intersection_id,
	const uint8_t& direction_of_src_intersection, const uint16_t& time_offset_step,
	const double& position_at_offset, const string& dst_node_id);

vector<vector<reference_wrapper<Car>>> choose_car_to_thread_group(vector<string> &new_car_ids, vector<string> &old_car_ids);
void delete_car_from_database(Car& car);

