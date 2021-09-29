#include <iostream>
#include <sstream>
#include <string>
#include <fstream>
#include <cmath>
#include <algorithm>
#include <queue>
#include "LaneAdviser.h"
#include "Intersection_manager.h"
#include "server.h"
#include "json.hpp"
#include "thread_worker.h"
#include <chrono>	// Measure runtime

using namespace std;
using json = nlohmann::json;


SUMO_Traci traci;
uint16_t _N_TIME_STEP;
float _TIME_STEP;
uint8_t _CHOOSE_CAR_OPTION;
uint8_t _TOP_N_CONGESTED;
uint8_t _THREAD_NUM;
string _ARRIVAL_RATE;
string _RANDOM_SEED;
uint8_t _ITERATION_NUM;
string _CAR_TIME_ERROR;

unordered_map<string, string> src_dst_dict;

void initial_sumo();
void read_src_dst_file(string src_dst_file_name);
void run_sumo();
string routing(unordered_map<string, Car_Info>& cars_info, string car_id);
void update_network_delay(unordered_map<string, Car_Info>& cars_info_dict); // Update network info at each time step

int main(int argc, char* argv[])
{
    cout << "Usage: ./main <grid_size> <src_dst_file.json> <_N_TIME_STEP> <_TIME_STEP> <_TOP_N_CONGESTED> <_CHOOSE_CAR_OPTION> <_THREAD_NUM> <_ITERATION_NUM> <_CAR_TIME_ERROR> <_ARRIVAL_RATE> <_RANDOM_SEED>" << endl;
    string src_dst_file_name = "data/routes/";
    // Parse the input
    if (argc >= 5) {
        string grid_size_str = argv[1];
        _grid_size = stoi(grid_size_str);
        src_dst_file_name += argv[2];
        _N_TIME_STEP = stoi(argv[3]);
        _TIME_STEP = stof(argv[4]);
        _TOP_N_CONGESTED = stoi(argv[5]);
        _CHOOSE_CAR_OPTION = stoi(argv[6]);
        _THREAD_NUM = stoi(argv[7]);
        _ITERATION_NUM = stoi(argv[8]);
        _CAR_TIME_ERROR = argv[9];
        _ARRIVAL_RATE = argv[10];
        _RANDOM_SEED = argv[11];
    }
    else {
        cout << "Wrong number of arguments" << endl;
        exit(-1);
    }

    // Read src_dst_dict
    read_src_dst_file(src_dst_file_name);

    // Initialization
    //initial_client_handler();
    initial_sumo();

    read_load_adv_data();
    read_inter_info_data();
    read_inter_length_data();
    //Thread_Worker router_thread;     // New thread to send/receive routing requests/results  (do this after initial_client_handler();)

    // SUMO simulation
    run_sumo();


    // TODO: get result from router first before updating the str
    traci.close();

    return 0;
}

// Initial SUMO
void initial_sumo() {
    traci.connect("localhost", SUMO_PORT);
}

// Read the source/destination file generated by python
void read_src_dst_file(string src_dst_file_name) {
    ifstream file(src_dst_file_name);

    if (!file.is_open()) {
        cout << "Fail to open the src_dst file in data/routes/" << endl;
        exit(-1);
    }

    json data;
    file >> data;
    file.close();

    for (const auto& src_dst_data_map : data.items()) {
        string car_id = src_dst_data_map.key();
        tuple<uint16_t, uint16_t, string> src_dst_data = src_dst_data_map.value();
        src_dst_dict[car_id] = get<2>(src_dst_data);
    }
}

void run_sumo() {
    double simu_step = 0;
    

    // Create a list with intersection managers
    map<My_Coord, IntersectionManager*> intersection_map;
    for (uint16_t i = 1; i <= _grid_size; i++) {
        for (uint16_t j = 1; j <= _grid_size; j++) {
            My_Coord coordinate(i, j);
            IntersectionManager* intersection = new IntersectionManager(coordinate);
            intersection_map[coordinate] = intersection;
        }
    }

    // Connect intersections for spillback prevention
    for (uint16_t i = 1; i <= _grid_size; i++) {
        for (uint16_t j = 1; j <= _grid_size; j++) {
            My_Coord coordinate(i, j);
            if (i <= _grid_size - 1) {
                My_Coord target_coordinate(i + 1, j);
                (*(intersection_map[coordinate])).connect(1, (*(intersection_map[target_coordinate])), 3);
            }
            if (j <= _grid_size - 1) {
                My_Coord target_coordinate(i, j + 1);
                (*(intersection_map[coordinate])).connect(2, (*(intersection_map[target_coordinate])), 0);
            }
        }
    }
    unordered_map<string, Car_Info> car_info_dict;            // Record car states
    vector<string> to_delete_car_in_database;     // Car exit the network

    vector<string> first_1000_car_id; // First 1000 cars
    unordered_map<string, double> all_travel_time;      // Record the travel time for experimental evaluation
    unordered_map<string, double> all_delay_time;      // Record the travel delay time for experimental evaluation
    unordered_map<string, double> all_shortest_time;      // Record the travel delay time for experimental evaluation
    unordered_map<string, double> all_diff_exit_time;      // Record the difference between estimated travel time and actual travel time
    int arrival_car_num = 0;

    // Start simulation
    while (traci.simulation.getMinExpectedNumber() > 0) {
        // Early terminate the simulation
        if (int(simu_step * 10) / 10.0 >= _N_TIME_STEP) {
            break;
        }

        traci.simulationStep();
        vector<string> all_c = traci.vehicle.getIDList();
        // Send route requests

        /*
        if (fmod(simu_step, ROUTING_PERIOD) < _TIME_STEP) {
            // Start routing


            string server_send_str = "";
            server_send_str += to_string(simu_step) + ";";
            for (auto& [car_id, car_info] : car_info_dict) {
                IntersectionManager* intersection_manager_ptr = car_info.intersection_manager_ptr;
                if (intersection_manager_ptr != nullptr) {
                    IntersectionManager& intersection_manager = *intersection_manager_ptr;

                    Car_Info_In_Intersection car_data = intersection_manager.get_car_info_for_route(car_id);
                    if (car_data.is_skip_car == false) {
                        double position_at_offset = car_data.position_at_offset;
                        uint16_t time_offset_step = car_data.time_offset_step;
                        string src_intersection_id = car_data.src_intersection_id;
                        uint8_t direction_of_src_intersection = car_data.direction_of_src_intersection;
                        uint8_t src_shift_num = car_data.src_shift_num;
                        car_info.src_shift_num = src_shift_num;

                        if (src_intersection_id.compare(car_info.dst_node_idx) != 0) {
                            server_send_str += car_id + ",";
                            server_send_str += car_info.route_state + ",";
                            server_send_str += to_string(car_info.car_length) + ",";
                            server_send_str += src_intersection_id + ",";
                            server_send_str += to_string(direction_of_src_intersection) + ",";
                            server_send_str += to_string(time_offset_step) + ",";               // Step that the datacenter needs to take
                            server_send_str += get_string_from_double(position_at_offset, 2) + ",";       // The position at the specific time
                            server_send_str += car_info.dst_node_idx + ";";

                        }
                        else {
                            server_send_str += car_id + ",";
                            server_send_str += string("LEAVING") + ";";
                        }
                    }
                    else {
                        server_send_str += car_id + ",";
                        server_send_str += string("PAUSE") + ";";
                    }
                }
            }

            for (const auto& car_id : to_delete_car_in_database) {
                server_send_str += car_id + ",";
                server_send_str += string("EXIT") + ";";
            }

            to_delete_car_in_database.clear();
            router_thread.route_request = server_send_str;
            // Send route request
            {
                unique_lock<mutex> worker_lock(router_thread.request_worker_mutex);
                router_thread.request_worker_ready = true;
                router_thread.request_worker_condition_variable.notify_all();
            }
            //cout << server_send_str << endl;
            // 
            // 
            // Wait for results
            {
                unique_lock<mutex> main_thread_lock(router_thread.routing_done_mutex);
                Thread_Worker* router_thread_ptr = &router_thread;
                router_thread.routing_done_condition_variable.wait(main_thread_lock, [router_thread_ptr] {return router_thread_ptr->allow_main_continue; });
                router_thread.allow_main_continue = false;
            }
            // Get result
            string route_result = router_thread.route_result;
            if (route_result.length() > 0) {
                route_result.pop_back();    // Remove the ";" at the end
                stringstream ss_car(route_result);
                while (ss_car.good()) {
                    string car_str;
                    getline(ss_car, car_str, ';');
                    stringstream ss_car_data(car_str);
                    string car_id;
                    getline(ss_car_data, car_id, ',');
                    string car_path;
                    getline(ss_car_data, car_path, ',');
                    string car_estimated_travel_time;
                    getline(ss_car_data, car_estimated_travel_time, ',');
                    car_info_dict[car_id].estimated_exit_time = stod(car_estimated_travel_time);

                    // Update the path
                    uint8_t src_shift_num = car_info_dict[car_id].src_shift_num;
                    string pre_route_part = "";
                    if (car_info_dict[car_id].route.length() > src_shift_num) {
                        pre_route_part = car_info_dict[car_id].route.substr(0, src_shift_num);
                    }
                    car_info_dict[car_id].route = pre_route_part + car_path + "SS";     //Add "next turn" at the end

                    //if (car_info_dict[car_id].route_state == "NEW" && car_info_dict[car_id].intersection_manager_ptr->car_list[car_id]->position <= TOTAL_LEN) {
                        //cout << "warning!!! " << car_id << " " << car_info_dict[car_id].intersection_manager_ptr->car_list[car_id]->position << endl;
                    //}

                    car_info_dict[car_id].route_state = "OLD";
                }

                //cout << route_result << endl;
            }
        }
        */

        update_network_delay(car_info_dict);
        // Update the position of each car
        for (const string& car_id : all_c) {
            string lane_id = traci.vehicle.getLaneID(car_id);

            // No record of the car
            if (car_info_dict.find(car_id) == car_info_dict.end()) {
                car_info_dict[car_id].route_state = "NEW";
                string dst_node_str = src_dst_dict[car_id];
                car_info_dict[car_id].intersection_manager_ptr = nullptr;

                car_info_dict[car_id].dst_node_idx = dst_node_str;
                car_info_dict[car_id].enter_time = simu_step;
                car_info_dict[car_id].car_length = uint8_t(traci.vehicle.getLength(car_id));
                car_info_dict[car_id].src_shift_num = 0;

                car_info_dict[car_id].compute_shortest_time(lane_id, dst_node_str);
                arrival_car_num++;

                string new_route = routing(car_info_dict, car_id);
                car_info_dict[car_id].route = new_route;

                if (first_1000_car_id.size() <= 1000) {
                    first_1000_car_id.push_back(car_id);
                }
            }
            // Hnadle the car in each intersection
            bool is_handled = false;
            for (auto& [intersection_id, intersection_ptr] : intersection_map) {
                const string& inter_region = intersection_ptr->check_in_my_region(lane_id);
                if (inter_region == "On my lane") {
                    // Check if the car enter the intersection (by changing state from "In intersection" to "on my lane")
                    if (car_info_dict[car_id].inter_status == "In my intersection") {
                        car_info_dict[car_id].route = car_info_dict[car_id].route.erase(0, 1);
                    }

                    char current_turn = car_info_dict[car_id].route[0];
                    char next_turn = car_info_dict[car_id].route[1];

                    intersection_ptr->update_car(car_id, lane_id, simu_step, current_turn, next_turn);
                    
                    is_handled = true;
                    car_info_dict[car_id].inter_status = "On my lane";
                    car_info_dict[car_id].intersection_manager_ptr = intersection_ptr;
                }
                else if (inter_region == "In my intersection") {
                    // Check if the car enter the intersection (by changing state from "On my lane" to "in intersection")
                    //if (car_info_dict[car_id].inter_status == "On my lane") {
                    //    car_info_dict[car_id].route = car_info_dict[car_id].route.erase(0, 1);
                    //}

                    char current_turn = car_info_dict[car_id].route[0];
                    char next_turn = car_info_dict[car_id].route[1];

                    intersection_ptr->update_car(car_id, lane_id, simu_step, current_turn, next_turn);

                    is_handled = true;
                    car_info_dict[car_id].inter_status = "In my intersection";
                    car_info_dict[car_id].intersection_manager_ptr = intersection_ptr;
                }
                else {
                    // The intersection doesn't have the car
                    intersection_ptr->delete_car(car_id);
                }
            }

            if (!is_handled) {
                // Leaving intersections
                traci.vehicle.setSpeed(car_id, V_MAX);
                car_info_dict[car_id].inter_status = "None";
                car_info_dict[car_id].intersection_manager_ptr = nullptr;
            }
        }

        // Remove cars
        vector<string> car_to_delete;
        for (const auto& [car_id, Car_Info] : car_info_dict) {
            // car_id is not found in current simulation
            if (find(all_c.begin(), all_c.end(), car_id) == all_c.end()) {
                car_to_delete.push_back(car_id);
            }
        }
        for (const auto& car_id : car_to_delete) {
            double car_travel_time = simu_step - car_info_dict[car_id].enter_time;
            all_travel_time[car_id] = car_travel_time;
            all_delay_time[car_id] = car_travel_time - car_info_dict[car_id].shortest_travel_time;
            all_shortest_time[car_id] = car_info_dict[car_id].shortest_travel_time;
            all_diff_exit_time[car_id] = simu_step - (car_info_dict[car_id].estimated_exit_time);
                        
            car_info_dict.erase(car_id);
            to_delete_car_in_database.push_back(car_id);    // This is for telling the router to delete the car
        }
        auto begin = chrono::high_resolution_clock::now();
        for (auto& [intersection_id, intersection_manager_ptr] : intersection_map) {
            intersection_manager_ptr->run(simu_step);
        }
        auto end = chrono::high_resolution_clock::now();

        auto route_time = chrono::duration<double>(end - begin);
        //cout << "Route_time: " << route_time.count() << " seconds" << endl;

        simu_step += _TIME_STEP;

        if (traci.simulation.getEndingTeleportNumber() > 0) {
            cout << "Collision warning!!" << endl;
            //system("Pause");
        }
    }

    // Statistics
    string file_name_prefix = string("result/") + to_string(_grid_size) + "_" + 
        to_string(_TOP_N_CONGESTED) + "_" + to_string(_CHOOSE_CAR_OPTION) + "_" + 
        to_string(_THREAD_NUM) + "_" + to_string(_ITERATION_NUM) + "_" + _CAR_TIME_ERROR + "_" + _ARRIVAL_RATE + "_" + _RANDOM_SEED + "_";
    ofstream all_car_file(file_name_prefix + "allCars.csv");
    ofstream statistic_file("result/statistic.csv", ofstream::app);

    all_car_file << "ID, shortest_travel_time, travel_time, delay_time, simu_diff_exit_time" << endl;
    double avg_travel_time = 0;
    double avg_delay_time = 0;
    double avg_diff_exit_time = 0;
    double avg_shortest_travel_time = 0;

    for (auto& [car_id, travel_time] : all_travel_time) {
        avg_shortest_travel_time += all_shortest_time[car_id];
        avg_travel_time += travel_time;
        avg_delay_time += all_delay_time[car_id];
        avg_diff_exit_time += all_diff_exit_time[car_id];

        all_car_file << car_id << ',' << all_shortest_time[car_id] << ',' << travel_time << ',' << all_delay_time[car_id] << ',' << all_diff_exit_time[car_id] << endl;
    }

    avg_travel_time /= all_travel_time.size();
    avg_delay_time /= all_delay_time.size();
    avg_diff_exit_time /= all_diff_exit_time.size();
    avg_shortest_travel_time /= all_shortest_time.size();

    double avg_first_1000_travel_time = 0;
    double avg_first_1000_delay_time = 0;
    double avg_first_1000_shortest_travel_time = 0;
    for (auto& car_id : first_1000_car_id) {
        avg_first_1000_shortest_travel_time += all_shortest_time[car_id];
        avg_first_1000_travel_time += all_travel_time[car_id];
        avg_first_1000_delay_time += all_delay_time[car_id];
    }
    avg_first_1000_travel_time /= first_1000_car_id.size();
    avg_first_1000_delay_time /= first_1000_car_id.size();
    avg_first_1000_shortest_travel_time /= first_1000_car_id.size();

    statistic_file << "Grid size, top N, choose_car, thread_num, iteration_num, \
        _CAR_TIME_ERROR, arrival_rate, rand_seed, avg_shortest_travel_time, avg_travel, \
        avg_delay, arrival_car_num, departured_car_num, diff_exit_time, 1000_travel_time, \
        1000_delay_time, 1000_shortest_travel_time, 1000_car_num" << endl;
    statistic_file << (int)_grid_size << ',' << (int)_TOP_N_CONGESTED << ',' << (int)_CHOOSE_CAR_OPTION << ',' << (int)_THREAD_NUM << ',' << (int)_ITERATION_NUM << ',' << _CAR_TIME_ERROR << ',';
    statistic_file << _ARRIVAL_RATE << ',' << _RANDOM_SEED << ',' << avg_shortest_travel_time << ',' << avg_travel_time << ',' << avg_delay_time << ',' << arrival_car_num << ',' << all_travel_time.size() << ',' << avg_diff_exit_time << ',';
    statistic_file << avg_first_1000_travel_time << ',' << avg_first_1000_delay_time << ',' << avg_first_1000_shortest_travel_time << ',' << first_1000_car_id.size() << endl;
    
    all_car_file.close();
    statistic_file.close();


    // Delete the intersection manager
    for (const auto& [intersection_id, manager_ptr] : intersection_map) {
        delete manager_ptr;
    }
}


typedef std::tuple<int, int> Coord; // (x, y)
typedef std::tuple<int, int, int> Node_ID; // (x, y, direction)
map<Node_ID, double> lane_delay_record;    // A global record is needed to prevent all-stop spillback blockage
void update_network_delay(unordered_map<string, Car_Info>& cars_info_dict) {
    // Delay on lanes
    map<Node_ID, double> lane_delay;
    map<Node_ID, double> lane_count_car;

    // Collect information from the map
    for (auto& [car_id, car_info] : cars_info_dict) {
        // Check if the car is scheduled
        IntersectionManager* intersection_manager_ptr = car_info.intersection_manager_ptr;

        // Skip if the car is not in any intersection
        if (intersection_manager_ptr == nullptr) {
            continue;
        }

        map<string, Car*>& car_list = intersection_manager_ptr->car_list;
        if (car_list.find(car_id) != car_list.end()) {
            Car* car_ptr = car_list[car_id];
            // Count only if scheduled
            if (car_ptr->is_scheduled) {
                string lane_id = traci.vehicle.getLaneID(car_id);

                if (lane_id[0] == ':') {
                    // Skip the case where the car is inside the intersection
                    continue;
                }
                lane_id = lane_id.substr(0,9);
                Node_ID node_id(stoi(lane_id.substr(0, 3)), stoi(lane_id.substr(4, 3)), (int)(lane_id[8]-'0'));
                if (lane_delay.find(node_id) != lane_delay.end()) {
                    // Found the lane in the record
                    lane_delay[node_id] += car_ptr->D;
                    lane_count_car[node_id] += 1;
                }
                else {
                    // Not found the lane in the record
                    lane_delay[node_id] = car_ptr->D;
                    lane_count_car[node_id] = 1;
                }
            }
        }
    }

    // Calculate the average delay
    for (auto& [node_id, added_delay] : lane_delay) {
        lane_delay_record[node_id] = added_delay / lane_count_car[node_id];
        //cout << get<0>(node_id) << '_' << get<1>(node_id) << '_' << get<2>(node_id) << "  " << lane_delay_record[node_id] << endl;
    }
}

class Node_in_Heap {
public:
    uint16_t current_arrival_time = 0;
    Node_ID current_node;
    Node_in_Heap() {}
    Node_in_Heap(uint16_t arrival_time, Node_ID node) : current_arrival_time(arrival_time), current_node(node) {}
};
struct Compare_AT {
    bool operator()(Node_in_Heap const& node1, Node_in_Heap const& node2) {
        return node1.current_arrival_time > node2.current_arrival_time;
    }
};
class Node_Record {
public:
    bool is_src = false;
    double arrival_time = 0;		// Time that the car arrives
    Node_ID last_intersection_id;
    char turning = 'S';

    Node_Record() {}
    Node_Record(bool in_is_src, double arrival_time_stamp) : is_src(in_is_src), arrival_time(arrival_time_stamp) {}
};
map<char, Node_ID> decide_available_turnings(Coord src_coord, uint8_t src_intersection_direction, Coord dst_coord, uint16_t additional_search_range);
string routing(unordered_map<string, Car_Info>& cars_info_dict, string target_car_id) {

    Car_Info& target_car_info = cars_info_dict[target_car_id];

    // Get the coordination of the destination 
    string& dst_node_str = target_car_info.dst_node_idx;
    int dst_x = stoi(dst_node_str.substr(0, 3));
    int dst_y = stoi(dst_node_str.substr(4, 3));
    const Coord dst_coord(dst_x, dst_y);

    // Get the coordination of source node 
    string lane_id = traci.vehicle.getLaneID(target_car_id);
    int src_x = stoi(lane_id.substr(0, 3));
    int src_y = stoi(lane_id.substr(4, 3));
    int src_dir = int(4 - (lane_id[8] - '0'));  // TODO: double check
    const Node_ID src_node(src_x, src_y, src_dir);

    // Initialization
    map<Node_ID, Node_Record> nodes_arrival_time_data;
    vector<Node_ID> visited_nodes;		// Record the visisted node
    priority_queue<Node_in_Heap, vector<Node_in_Heap>, Compare_AT > unvisited_queue;
    
    nodes_arrival_time_data[src_node] = Node_Record(true, 500.0 / V_MAX);    // "True" for src_node. Free speed traveling to the first node
    if (lane_delay_record.find(src_node) != lane_delay_record.end()) {
        nodes_arrival_time_data[src_node].arrival_time += lane_delay_record[src_node];   // Add delay on the road
    }

    // Push the src node into the queue
    Node_in_Heap src_node_in_heap(nodes_arrival_time_data[src_node].arrival_time, src_node);
    unvisited_queue.push(src_node_in_heap);

    Node_ID dst_node;   // Record the destination node

    // Routing
    while (unvisited_queue.size() > 0) {
        Node_in_Heap node_in_heap = unvisited_queue.top();
        unvisited_queue.pop();

        uint16_t current_arrival_time = node_in_heap.current_arrival_time;
        Node_ID current_node = node_in_heap.current_node;

        // Skip if the node is visited, prevent multiple push into the heap
        if (find(visited_nodes.begin(), visited_nodes.end(), current_node) != visited_nodes.end()) {
            continue;
        }

        // Mark current node as "visisted"
        visited_nodes.push_back(current_node);

        int& intersection_x = get<0>(current_node);
        int& intersection_y = get<1>(current_node);
        int& intersection_dir = get<2>(current_node);
        Coord intersection_id(intersection_x, intersection_y);

        // Terminate when finding shortest path
        if (dst_coord == intersection_id) {
            dst_node = current_node;
            break;
        }

        // Decide the available turnings
        map<char, Node_ID> available_turnings_and_out_direction = decide_available_turnings(intersection_id, intersection_dir, dst_coord, 0);

        for (pair<char, Node_ID> const& data_pair : available_turnings_and_out_direction) {
            char turning = data_pair.first;
            Node_ID node_id = data_pair.second;

            int next_time_step = 0;
            if (Coord(get<0>(node_id), get<1>(node_id)) == dst_coord) {
                // Road to the destination
                next_time_step = 500.0 / V_MAX;;    // Free speed traveling
            }
            else {
                next_time_step = 200.0 / V_MAX;;    // Free speed traveling
            }
            // Add delay on the road
            if (lane_delay_record.find(node_id) != lane_delay_record.end()) {
                next_time_step += lane_delay_record[node_id];
            }

            if ((nodes_arrival_time_data.find(node_id) == nodes_arrival_time_data.end())
                || (nodes_arrival_time_data[node_id].arrival_time > next_time_step)) {
                // Update the record with the new candidate
                Node_Record tmp_node_record(false, next_time_step);
                tmp_node_record.turning = turning;
                tmp_node_record.last_intersection_id = current_node;
                nodes_arrival_time_data[node_id] = tmp_node_record;

                // Push next node into the queue
                Node_in_Heap node_in_heap(next_time_step, node_id);
                unvisited_queue.push(node_in_heap);
            }
        }
    }

    // Retrieve the path
    string path_list = "";
    Node_Record node_data = nodes_arrival_time_data[dst_node];

    while (node_data.is_src == false) {
        path_list.insert(0, 1, node_data.turning);  // insert the turning into the path
        node_data = nodes_arrival_time_data[node_data.last_intersection_id];
    }

    return path_list;
}


map<char, Node_ID> decide_available_turnings(Coord src_coord, uint8_t src_intersection_direction, Coord dst_coord, uint16_t additional_search_range) {
    // additional_search_range : additional intersection number to be searched
    // Value of the dict : id of next intersection, the direction of next intersection
    map<char, Node_ID> available_turnings_and_out_direction;
    if (src_intersection_direction == 0) {
        if (get<0>(dst_coord) - get<0>(src_coord) > -additional_search_range) {
            if (get<0>(src_coord) < _grid_size || get<1>(src_coord) == get<1>(dst_coord)) {
                available_turnings_and_out_direction['R'] = Node_ID(get<0>(src_coord) + 1, get<1>(src_coord), 3);
            }
        }

        if (get<0>(src_coord) - get<0>(dst_coord) > -additional_search_range) {
            if (get<0>(src_coord) > 1 || get<1>(src_coord) == get<1>(dst_coord)) {
                available_turnings_and_out_direction['L'] = Node_ID(get<0>(src_coord) - 1, get<1>(src_coord), 1);
            }
        }

        if (get<1>(dst_coord) - get<1>(src_coord) > -additional_search_range) {
            if (get<1>(src_coord) < _grid_size || get<0>(src_coord) == get<0>(dst_coord)) {
                available_turnings_and_out_direction['S'] = Node_ID(get<0>(src_coord), get<1>(src_coord) + 1, 0);
            }
        }
    }
    else if (src_intersection_direction == 1) {
        if (get<1>(dst_coord) - get<1>(src_coord) > -additional_search_range) {
            if (get<1>(src_coord) < _grid_size || get<0>(src_coord) == get<0>(dst_coord)) {
                available_turnings_and_out_direction['R'] = Node_ID(get<0>(src_coord), get<1>(src_coord) + 1, 0);
            }
        }
        if (get<1>(src_coord) - get<1>(dst_coord) > -additional_search_range) {
            if (get<1>(src_coord) > 1 || get<0>(src_coord) == get<0>(dst_coord)) {
                available_turnings_and_out_direction['L'] = Node_ID(get<0>(src_coord), get<1>(src_coord) - 1, 2);
            }
        }
        if (get<0>(src_coord) - get<0>(dst_coord) > -additional_search_range) {
            if (get<0>(src_coord) > 1 || get<1>(src_coord) == get<1>(dst_coord)) {
                available_turnings_and_out_direction['S'] = Node_ID(get<0>(src_coord) - 1, get<1>(src_coord), 1);
            }
        }
    }
    else if (src_intersection_direction == 2) {
        if (get<0>(src_coord) - get<0>(dst_coord) > -additional_search_range) {
            if (get<0>(src_coord) > 1 || get<1>(src_coord) == get<1>(dst_coord)) {
                available_turnings_and_out_direction['R'] = Node_ID(get<0>(src_coord) - 1, get<1>(src_coord), 1);
            }
        }
        if (get<0>(dst_coord) - get<0>(src_coord) > -additional_search_range) {
            if (get<0>(src_coord) < _grid_size || get<1>(src_coord) == get<1>(dst_coord)) {
                available_turnings_and_out_direction['L'] = Node_ID(get<0>(src_coord) + 1, get<1>(src_coord), 3);
            }
        }
        if (get<1>(src_coord) - get<1>(dst_coord) > -additional_search_range) {
            if (get<1>(src_coord) > 1 || get<0>(src_coord) == get<0>(dst_coord)) {
                available_turnings_and_out_direction['S'] = Node_ID(get<0>(src_coord), get<1>(src_coord) - 1, 2);
            }
        }
    }
    else if (src_intersection_direction == 3) {
        if (get<1>(src_coord) - get<1>(dst_coord) > -additional_search_range) {
            if (get<1>(src_coord) > 1 || get<0>(src_coord) == get<0>(dst_coord)) {
                available_turnings_and_out_direction['R'] = Node_ID(get<0>(src_coord), get<1>(src_coord) - 1, 2);
            }
        }
        if (get<1>(dst_coord) - get<1>(src_coord) > -additional_search_range) {
            if (get<1>(src_coord) < _grid_size || get<0>(src_coord) == get<0>(dst_coord)) {
                available_turnings_and_out_direction['L'] = Node_ID(get<0>(src_coord), get<1>(src_coord) + 1, 0);
            }
        }
        if (get<0>(dst_coord) - get<0>(src_coord) > -additional_search_range) {
            if (get<0>(src_coord) < _grid_size || get<1>(src_coord) == get<1>(dst_coord)) {
                available_turnings_and_out_direction['S'] = Node_ID(get<0>(src_coord) + 1, get<1>(src_coord), 3);
            }
        }
    }

    return available_turnings_and_out_direction; // Key : turnings, Values : out_direction
}
