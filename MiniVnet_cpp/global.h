#include <map>
#include <tuple>
#include <vector>
#include "element.h"
using namespace std;



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
void create_grid_network();
void add_time_step();
void move_a_time_step();