#include <map>
#include <tuple>
#include <vector>
#include <string>
using namespace std;

class Intersection {
public:
	tuple<int, int> id;
	uint16_t AZ_accumulated_size = 0;
	uint16_t GZ_accumulated_size = 0;

	Intersection() {}
	Intersection(tuple<int, int> &in_coordinate);

};

class Car_in_database {
public:
	string id = "";
	uint8_t lane = 0;
	uint8_t length = 0;
	char current_turn = 'S';
	uint16_t position = 0;


	// temporary variables for routing
	double D = 0;
	double OT = 0;
	uint8_t dst_lane = 0;
	uint8_t dst_lane_changed_to = 0;
	double speed_in_intersection = 0;

	bool is_spillback = false;
	bool is_spillback_strict = false;


	Car_in_database() {};
	Car_in_database(string in_id, uint8_t in_length);

	void update_dst_lane_and_data();
};

class Car : public Car_in_database {
public:
	tuple<int, int> dst_coord;
	vector<string, Intersection> records_intersection_in_database;
	
	// temporary variables for routing
	tuple<int, int>src_coord;
	uint8_t direction_of_src_intersection = 0;
	uint16_t time_offset_step = 0;
	double position_at_offset = 0;

	double traveling_time = 0;

	//self.path_data = None
	//self.dst_node = None    # record the dst node after routing
};