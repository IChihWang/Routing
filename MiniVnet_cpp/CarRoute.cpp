#include "global.h" 
map< Node_ID, double> _avg_map_delay;

void initial_avg_map() {
	for (int i = 1; i <= _grid_size; i++) {
		for (int j = 1; j <= _grid_size; j++) {
			for (int k = 0; k < 4; k++) {
				_avg_map_delay[Node_ID(Coord(i, j),k)] = 0;
			}
		}
	}
}


void compute_avg_map() {
	// Clear the database
	for (int i = 1; i <= _grid_size; i++) {
		for (int j = 1; j <= _grid_size; j++) {
			for (int k = 0; k < 4; k++) {
				_avg_map_delay[Node_ID(Coord(i, j), k)] = 0;
			}
		}
	}

	size_t time_size_in_database = _database.size();
	for (map< Coord, Intersection* >* intersection_map : _database) {
		for (int i = 1; i <= _grid_size; i++) {
			for (int j = 1; j <= _grid_size; j++) {
				Intersection* intersection = (*intersection_map)[Coord(i, j)];
				double link_delay[4] = {0};		// Four direction
				double link_car_count[4] = {0};	// Car count on the four links
				for (auto &[car_id, car_in_database] : *(intersection->sched_cars)) {
					uint8_t link = car_in_database.lane / LANE_NUM_PER_DIRECTION;
					link_delay[link] += car_in_database.D + car_in_database.OT;
					link_car_count[link]++;
				}
				for (int k = 0; k < 4; k++) {
					link_delay[k] /= link_car_count[k];
					_avg_map_delay[Node_ID(Coord(i, j), k)] += link_delay[k] / time_size_in_database;
				}
			}
		}
	}
}

class BR_Node_in_Heap {
public:
	double current_arrival_time = 0;
	Node_ID current_node;

	BR_Node_in_Heap() {}
	BR_Node_in_Heap(double arrival_time, Node_ID node) : current_arrival_time(arrival_time), current_node(node) {}
};
struct BR_Compare_AT {
	bool operator()(BR_Node_in_Heap const& node1, BR_Node_in_Heap const& node2) {
		return node1.current_arrival_time > node2.current_arrival_time;
	}
};
void brief_route() {
	for (auto& [car_id, car] : _car_dict) {
		// Dijkstra's Algorithm
		const Coord& dst_coord = car.dst_coord;
		Node_ID src_node(car.src_coord, car.direction_of_src_intersection);
		vector<Node_ID> visited_nodes;		// Record the visisted node
		priority_queue<BR_Node_in_Heap, vector<BR_Node_in_Heap>, BR_Compare_AT > unvisited_queue;
		BR_Node_in_Heap src_node_in_heap(0, src_node);
		unvisited_queue.push(src_node_in_heap);
		map<Node_ID, double> nodes_arrival_time_data;
		map<Node_ID, Node_ID> nodes_last_node;
		nodes_arrival_time_data[src_node] = 0;
		nodes_last_node[src_node] = src_node;

		Node_ID dst_node;

		// Routing
		while (unvisited_queue.size() > 0) {
			BR_Node_in_Heap node_in_heap = unvisited_queue.top();
			unvisited_queue.pop();
			double current_arrival_time = node_in_heap.current_arrival_time;
			Node_ID current_node = node_in_heap.current_node;

			// Skip if the node is visited, prevent multiple push into the heap
			if (find(visited_nodes.begin(), visited_nodes.end(), current_node) != visited_nodes.end()) {
				continue;
			}

			// Mark current node as "visisted"
			visited_nodes.push_back(current_node);

			Coord intersection_id = get<0>(current_node);
			uint8_t intersection_dir = get<1>(current_node);

			// Decide the turnings
			map<char, Node_ID> available_turnings_and_out_direction = decide_available_turnings(intersection_id, intersection_dir, dst_coord, 0);

			bool is_reached_dst = false;
			for (pair<char, Node_ID> const& data_pair : available_turnings_and_out_direction) {
				Node_ID next_node_id = data_pair.second;

				// Terminate when finding shortest path
				if (get<0>(next_node_id) == car.dst_coord) {
					dst_node = next_node_id;
					nodes_last_node[next_node_id] = current_node;
					is_reached_dst = true;
					break;
				}

				double next_arrival_time = current_arrival_time + _avg_map_delay[next_node_id];

				if ((nodes_arrival_time_data.find(next_node_id) == nodes_arrival_time_data.end())
					|| (nodes_arrival_time_data[next_node_id] > next_arrival_time)) {
					nodes_last_node[next_node_id] = current_node;
					BR_Node_in_Heap node_in_heap(next_arrival_time, next_node_id);
					unvisited_queue.push(node_in_heap);
				}
			}

			if (is_reached_dst) {
				break;
			}
		}

		// Retrieve the paths
		vector<Coord> path_list;
		Node_ID current_node = dst_node;
		path_list.push_back(get<0>(dst_node));
		while (current_node != src_node) {
			current_node = nodes_last_node[current_node];
			path_list.insert(path_list.begin(), get<0>(current_node));
		}
		
		// Write to this
		car.brief_path_data = path_list;
	}
}